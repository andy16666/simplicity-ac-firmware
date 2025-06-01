/*
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
  Firmware for my Simplicity air conditioner. This unit was built in 2003 and 
  has a very simple MCU-based control board, with relays controlled via a sink 
  driver IC. I modified it, adding a Pi Pico W in order to interface it to 
  homebridge, and, ultimately, homekit. 
  
  GPIO outputs are directly wired to the inputs of the sink driver, 
  and the 4 original traces are cut. The Pi Pico W is powered from the onboard 
  +5V rail. 

  Add a config.h which defines WIFI_SSID, WIFI_PASS, and HOSTNAME. 

  The firmare uses an mDNS responder which will advertise the hostname as 
  hostname.local on the local network. 

  The firmare uses various state machines: 
  - A high level state governs the incoming command state 
  - A state machine for the A/C unit governs its current high level state
  - A fan state machine governs fan spee 
  - A compressor state machine governs the compressor state

  Author: Andrew Somerville <andy16666@gmail.com> 
  GitHub: andy16666
*/

#include <LEAmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <TemperatureSensors.h>
#include <CPU.h>
#include "config.h"
#include <SimplicityAC.h>
#include <util.h>
extern "C" {
  #include "threadkernel.h"
}

using namespace AOS; 

#define TEMP_SENSOR_PIN 2

#define PING_INTERVAL_MS 10000
#define SENSOR_READ_INTERVAL_MS 5000
#define MAX_CONSECUTIVE_FAILED_PINGS 5

// Active high outputs
#define COMPRESSOR_PIN 10
#define FAN_LOW_PIN    11
#define FAN_MED_PIN    12
#define FAN_HIGH_PIN   13

// Fan will satge down after turning off the unit from standby high -> med -> low -> off
// This allows the longer ducts I'm using to warm back up to room temperature and the coil
// to dry after a cooling phase. It also prevents the fan from cycling on and off which
// can be annoying at night. 1, 5 and 15 minutes for high, medium and then low keeps
// the fan running long enough that if cooling is needed again, it will probably not
// have stopped yet.
#define STANDBY_HIGH_TIME_MS 60000
#define STANDBY_MED_TIME_MS 300000
#define STANDBY_LOW_TIME_MS 900000

// Transition time for changes to fan speed and compressor state. This must be long enough
// for the circuit breaker to recover. 1s is much too short. 5s is close to the factory
// setting.
#define TRANSITION_TIME_MS 5000

// The fan uses 3 different windings, one for each speed. This delay allows the previous
// winding to de-energize before engaging the new one. Must be long enough to allow the
// magnetic field to dissipate but not so long that the fan stops. 100ms is a good starting
// point.
#define FAN_SPEED_TRANSITION_TIME_MS 100

#define COOL_HIGH_OFF_TEMP_C  1.5
#define COOL_HIGH_ON_TEMP_C 6

#define COOL_MED_OFF_TEMP_C   5
#define COOL_MED_ON_TEMP_C  12

#define COOL_LOW_OFF_TEMP_C  8
#define COOL_LOW_ON_TEMP_C  17

#define EVAP_TEMP_ADDR   61
#define OUTLET_TEMP_ADDR 20

CPU cpu; 

const char* STATUS_JSON_FORMAT = 
          "{\n"\
          "  \"evapTempC\":\"%s\",\n" \
          "  \"outletTempC\":\"%s\",\n" \
          "  \"command\":\"%c\",\n" \
          "  \"state\":\"%c\",\n" \
          "  \"compressorState\":\"%d\",\n" \
          "  \"fanState\":\"%c\",\n" \
          "  \"cpuTempC\":\"%f\",\n" \
          "  \"tempErrors\":%d,\n" \
          "  \"lastPing\":\"%fms\",\n" \
          "  \"consecutiveFailedPings\":%d,\n" \
          "  \"freeMem\":%d,\n" \
          "  \"powered\":\"%s\",\n" \
          "  \"booted\":\"%s\",\n" \
          "  \"connected\":\"%s\",\n" \
          "  \"numRebootsPingFailed\":%d,\n" \
          "  \"numRebootsDisconnected\":%d,\n" \
          "  \"wiFiStatus\":%d\n" \
          "}\n";

WebServer server(80);
TemperatureSensors TEMPERATURES(TEMP_SENSOR_PIN); 

// Internal states
volatile ac_state_t state = AC_POWER_OFF;
volatile compressor_state_t compressorState = AC_COMPRESSOR_OFF;
volatile fan_state_t fanState = AC_FAN_OFF;

static volatile long            initialize             __attribute__((section(".uninitialized_data")));
static volatile long            timeBaseMs             __attribute__((section(".uninitialized_data")));
static volatile long            powerUpTime            __attribute__((section(".uninitialized_data")));
static volatile long            tempErrors             __attribute__((section(".uninitialized_data")));
static volatile long            numRebootsPingFailed   __attribute__((section(".uninitialized_data")));
static volatile long            numRebootsDisconnected __attribute__((section(".uninitialized_data")));
static volatile ac_cmd_t        command                __attribute__((section(".uninitialized_data")));

volatile long lastStateChangeTimeMs = millis();
volatile long startupTime = millis();
volatile long connectTime = millis();
volatile unsigned long lastPingMicros = 0; 
volatile unsigned int consecutiveFailedPings = 0; 

threadkernel_t* CORE_0_KERNEL; 
threadkernel_t* CORE_1_KERNEL;

void setup() 
{
  Serial.setDebugOutput(false);

  if (initialize)
  {
    timeBaseMs = 0; 
    powerUpTime = millis(); 
    tempErrors = 0; 
    command = CMD_AC_OFF;
    numRebootsPingFailed = 0; 
    numRebootsDisconnected = 0; 
    initialize = 0; 
  }

  CORE_0_KERNEL = create_threadkernel(&millis); 
  CORE_1_KERNEL = create_threadkernel(&millis); 

  cpu.begin(); 

  wifi_connect();

  server.begin();

  if (MDNS.begin(HOSTNAME)) {
    Serial.println("MDNS responder started");
  }

  server.on("/", []() {
    for (uint8_t i = 0; i < server.args(); i++) {
      String argName = server.argName(i);
      String arg = server.arg(i);

      if (argName.equals("cmd") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'O':  command = CMD_AC_OFF;  break;
          case 'C':  command = CMD_AC_COOL_HIGH; break;  
          case 'H':  command = CMD_AC_COOL_HIGH; break;  
          case 'M':  command = CMD_AC_COOL_MED; break;  
          case 'L':  command = CMD_AC_COOL_LOW; break;  
          case 'K':  command = CMD_AC_KILL; break;  
          case 'F':  command = CMD_AC_FAN;  break; 
          default:   command = CMD_AC_OFF;  
        }
      }
    }
    {
      char buffer[1024];
      unsigned long timeMs = millis(); 
      sprintf(buffer, STATUS_JSON_FORMAT,
              TEMPERATURES.formatTempC(EVAP_TEMP_ADDR).c_str(),
              TEMPERATURES.formatTempC(OUTLET_TEMP_ADDR).c_str(),
              command,
              state,
              compressorState,
              fanState,
              cpu.getTemperature(),
              tempErrors,
              lastPingMicros/1000.0,
              consecutiveFailedPings,
              getFreeHeap(),
              msToHumanReadableTime((timeBaseMs + timeMs) - powerUpTime).c_str(), 
              msToHumanReadableTime(timeMs - startupTime).c_str(),
              msToHumanReadableTime(timeMs - connectTime).c_str(),
              numRebootsPingFailed,
              numRebootsDisconnected, 
              WiFi.status());
      server.send(200, "text/json", buffer);
    }
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_mdnsUpdate); 

  CORE_0_KERNEL->add(CORE_0_KERNEL, task_testPing, PING_INTERVAL_MS); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_testWiFiConnection, 100); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_handleClient, 0); 
}

void loop() 
{
  CORE_0_KERNEL->run(CORE_0_KERNEL); 
}

void setup1() 
{
  TEMPERATURES.add("Evap", EVAP_TEMP_ADDR); 
  TEMPERATURES.add("Outlet", OUTLET_TEMP_ADDR); 
  TEMPERATURES.discoverSensors();

  pinMode(COMPRESSOR_PIN, OUTPUT);
  digitalWrite(COMPRESSOR_PIN, LOW);

  pinMode(FAN_LOW_PIN, OUTPUT);
  digitalWrite(FAN_LOW_PIN, LOW);

  pinMode(FAN_MED_PIN, OUTPUT);
  digitalWrite(FAN_MED_PIN, LOW);

  pinMode(FAN_HIGH_PIN, OUTPUT);
  digitalWrite(FAN_HIGH_PIN, LOW);

  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_processCommans); 

  CORE_1_KERNEL->add(CORE_1_KERNEL, task_readTemperatures, SENSOR_READ_INTERVAL_MS);
}

void loop1()
{
  CORE_1_KERNEL->run(CORE_1_KERNEL); 
}

void task_handleClient()
{
  server.handleClient(); 
}

void task_mdnsUpdate()
{
  MDNS.update(); 
}

void task_testWiFiConnection()
{
  if (!is_wifi_connected()) 
  {
    Serial.println("WiFi disconnected, rebooting.");
    numRebootsDisconnected++; 
    reboot();  
  }
}

void task_testPing()
{
  unsigned long timeMicros = micros(); 
  int pingStatus = WiFi.ping(WiFi.gatewayIP(), 255); 
  lastPingMicros = micros() - timeMicros; 
  if (pingStatus < 0) 
  {
    if (++consecutiveFailedPings >= MAX_CONSECUTIVE_FAILED_PINGS)
    {
      Serial.println("Gateway unresponsive, rebooting.");
      numRebootsPingFailed++; 
      reboot(); 
    }
  }
  else 
  {
    consecutiveFailedPings = 0; 
  }
}

void task_processCommans()
{
  state = processCommand(state, command);
}

void task_readTemperatures()
{
  TEMPERATURES.readSensors();
}

void reboot()
{
  timeBaseMs += millis(); 
  rp2040.reboot(); 
}

ac_state_t changeState(const ac_state_t currentState, const ac_state_t targetState) {
  if (targetState != currentState) {
    Serial.printf("changeState: %c => %c\r\n", currentState, targetState);

    switch (targetState) {
      case AC_COOL_HIGH:
        {
          setFan(AC_FAN_HIGH);
          break;
        }
      case AC_COOL_MED:
        {
          setFan(AC_FAN_MED);
          break;
        }
      case AC_COOL_LOW:
        {
          setFan(AC_FAN_LOW);
          break;
        }
      case AC_STANDBY_HIGH:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_HIGH);
          break;
        }
      case AC_STANDBY_MED:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_MED);
          break;
        }
      case AC_STANDBY_LOW:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_LOW);
          break;
        }
      case AC_POWER_OFF:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_OFF);
          break;
        }
    }
    lastStateChangeTimeMs = millis();
  }

  return targetState;
}

int isEvaporatorInSafeTempRange() 
{
  switch(compressorState) 
  {
    case AC_COMPRESSOR_ON: return 
        TEMPERATURES.getTempC(EVAP_TEMP_ADDR) > 12.0 && TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) > 0.5;
    case AC_COMPRESSOR_OFF: return 
        TEMPERATURES.getTempC(EVAP_TEMP_ADDR) > 14.0 && TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) > 5.0;
    default: return 1; 
  }
}

float calculateOnTemp(float stateOffTempC, float stateOnTempC, float minRangeC, float minEvapDeltaC, float maxEvapDeltaC)
{
  // If the state on temp is larger than the evap temp, we are unlikely to ever cool. Adjust it to just under. 
  float maxOnTempC = fmin(TEMPERATURES.getTempC(EVAP_TEMP_ADDR) - minEvapDeltaC, stateOnTempC);

  // If we try to start with too large a delta, the compressor will see huge torque and fail to operate. 
  float minOnTempC = fmax(TEMPERATURES.getTempC(EVAP_TEMP_ADDR) - maxEvapDeltaC, stateOffTempC + minRangeC); 

  if (maxOnTempC > minOnTempC)
  {
    return maxOnTempC; 
  }
  else 
  {
    return minOnTempC; 
  }
}

ac_state_t processCommand(const ac_state_t currentState, const ac_cmd_t command) 
{
  ac_state_t newState = currentState;
  switch (command) 
  {
    case CMD_AC_COOL_HIGH:
    {
      float onTempC = calculateOnTemp(COOL_HIGH_OFF_TEMP_C, COOL_HIGH_ON_TEMP_C, 5.0, 5.0, 8.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case AC_STANDBY_HIGH: newState = AC_COOL_HIGH; break;
        case AC_COOL_HIGH: controlCompressorState(COOL_HIGH_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STANDBY_HIGH; break;
      }
      break;
    }
    case CMD_AC_COOL_MED:
    {
      float onTempC = calculateOnTemp(COOL_MED_OFF_TEMP_C, COOL_MED_ON_TEMP_C, 5.0, 2.0, 7.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case AC_STANDBY_MED: newState = AC_COOL_MED; break;
        case AC_COOL_MED: controlCompressorState(COOL_MED_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STANDBY_MED; break;
      }
      break;
    }
    case CMD_AC_COOL_LOW:
    {
      float onTempC = calculateOnTemp(COOL_LOW_OFF_TEMP_C, COOL_LOW_ON_TEMP_C, 5.0, 1.0, 5.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case AC_STANDBY_LOW: newState = AC_COOL_LOW; break;
        case AC_COOL_LOW: controlCompressorState(COOL_LOW_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STANDBY_LOW; break;
      }
      break;
    }
    case CMD_AC_OFF:
    {
      long elapsedStateTimeMs = millis() - lastStateChangeTimeMs;
      switch (currentState) 
      {
        case AC_COOL_HIGH: newState = AC_STANDBY_HIGH; break;
        case AC_COOL_MED:  newState = AC_STANDBY_MED;  break;
        case AC_COOL_LOW:  newState = AC_STANDBY_LOW;  break;
        case AC_STANDBY_HIGH: newState = elapsedStateTimeMs > STANDBY_HIGH_TIME_MS ? AC_STANDBY_MED : currentState; break;
        case AC_STANDBY_MED:  newState = elapsedStateTimeMs > STANDBY_MED_TIME_MS  ? AC_STANDBY_LOW : currentState; break;
        case AC_STANDBY_LOW:  newState = elapsedStateTimeMs > STANDBY_LOW_TIME_MS  ? AC_POWER_OFF   : currentState; break;
      }
      break;
    }
    case CMD_AC_KILL: 
    {
      switch (currentState) 
      {
        case AC_COOL_HIGH: newState = AC_STANDBY_HIGH; break;
        case AC_COOL_MED:  newState = AC_STANDBY_MED;  break;
        case AC_COOL_LOW:  newState = AC_STANDBY_LOW;  break;
        default: newState = AC_POWER_OFF; break;
      }
      break; 
    }
    case CMD_AC_FAN: newState = AC_STANDBY_HIGH; break;
  }

  if (newState != currentState) {
    Serial.printf("==> processCommand: %c: %d => %d\r\n", command, currentState, newState);
    newState = changeState(currentState, newState);
  }

  return newState;
}

void setFan(const fan_state_t targetState) 
{
  if (targetState != fanState) 
  {
    // Turn off the currently energized winding, if any
    switch (fanState) 
    {
      case AC_FAN_LOW:  digitalWrite(FAN_LOW_PIN, LOW); break;
      case AC_FAN_MED:  digitalWrite(FAN_MED_PIN, LOW); break;
      case AC_FAN_HIGH: digitalWrite(FAN_HIGH_PIN, LOW); break;
    }

    // If transitioning from another on state, delay a brief time to allow the winding to de-energize.
    if (fanState != AC_FAN_OFF && targetState != AC_FAN_OFF)
      delay(FAN_SPEED_TRANSITION_TIME_MS);

    // Energize the traget winding, if any
    switch (targetState) 
    {
      case AC_FAN_LOW: digitalWrite(FAN_LOW_PIN, HIGH); break;
      case AC_FAN_MED: digitalWrite(FAN_MED_PIN, HIGH); break;
      case AC_FAN_HIGH: digitalWrite(FAN_HIGH_PIN, HIGH); break;
    }

    Serial.printf("setFan: %c => %c\r\n", fanState, targetState);
    delay(TRANSITION_TIME_MS);
    fanState = targetState;
  }
}

void setCompressor(const compressor_state_t targetState) 
{
  if (targetState != compressorState) 
  {
    switch (targetState) 
    {
      case AC_COMPRESSOR_ON:  digitalWrite(COMPRESSOR_PIN, HIGH); break;
      case AC_COMPRESSOR_OFF: digitalWrite(COMPRESSOR_PIN, LOW); break;
    }

    Serial.printf("setCompressor: %d => %d\r\n", compressorState, targetState);
    delay(TRANSITION_TIME_MS);
    compressorState = targetState;
  }
}

void controlCompressorState(float offTempC, float onTempC) 
{
  switch(compressorState) 
  {
    case AC_COMPRESSOR_ON: 
    {
      if (TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) <= offTempC || !isEvaporatorInSafeTempRange())
        setCompressor(AC_COMPRESSOR_OFF); 
      break; 
    }
    case AC_COMPRESSOR_OFF: 
    {
      if (TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) >= onTempC && isEvaporatorInSafeTempRange())
        setCompressor(AC_COMPRESSOR_ON); 
      break; 
    }
  }
}

// WiFi

// WIFI

bool is_wifi_connected() 
{
  return WiFi.status() == WL_CONNECTED;
}

void wifi_connect() 
{
  WiFi.noLowPowerMode();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.noLowPowerMode();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 20;
  while (!is_wifi_connected() && (attempts--)) 
  {
    delay(1000);
  }

  if (!is_wifi_connected()) 
  {
    Serial.print("Connect failed! Rebooting.");
    numRebootsDisconnected++; 
    reboot(); 
  }

  Serial.println("Connected");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Hostname: ");
  Serial.print(HOSTNAME);
  Serial.println(".local");

  connectTime = millis();
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) 
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}
