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
#include "DS18B20.h"
#include "config.h"

#define TEMP_SENSOR_PIN 2

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

#define COOL_HIGH_OFF_TEMP_C  2
#define COOL_HIGH_ON_TEMP_C 6

#define COOL_MED_OFF_TEMP_C   5
#define COOL_MED_ON_TEMP_C  12

#define COOL_LOW_OFF_TEMP_C  10
#define COOL_LOW_ON_TEMP_C  17

#define EVAP_TEMP_ADDR   61
#define OUTLET_TEMP_ADDR 20

#define INVALID_TEMP FLT_MIN

#define EVAP_IDX    0
#define OUTLET_IDX  1
#define NUM_SENSORS 2

DS18B20 ds(TEMP_SENSOR_PIN);

const char* STATUS_JSON_FORMAT = 
            "{\n"\
            "  \"evapTempC\":\"%f\",\n" \
            "  \"outletTempC\":\"%f\",\n" \
            "  \"uptime\":%d,\n" \
            "  \"connected\":%d,\n" \
            "  \"command\":\"%c\",\n" \
            "  \"state\":\"%c\",\n" \
            "  \"compressorState\":\"%d\",\n" \
            "  \"fanState\":\"%c\",\n" \
            "  \"freeMem\":%d,\n" \
            "  \"wiFiStatus\":%d\n" \
            "}\n";
const pin_size_t TEMP_SENSOR_ADDRESSES[(NUM_SENSORS)] = {EVAP_TEMP_ADDR, OUTLET_TEMP_ADDR};
float            TEMPERATURES_C[(NUM_SENSORS)]    = {INVALID_TEMP, INVALID_TEMP};
const char*      SENSOR_NAMES[(NUM_SENSORS)] = {"Evap", "Outlet"}; 

WebServer server(80);

// External command to the unit
typedef enum {
  CMD_OFF = '-',
  CMD_COOL_HIGH = 'C',
  CMD_COOL_MED  = 'M',
  CMD_COOL_LOW  = 'L',
  CMD_KILL = 'K',
  CMD_FAN = 'F'
} ac_cmd_t;

// A/C state is changed based on the external command and the current state.
typedef enum {
  POWER_OFF = '-',
  COOL_HIGH = 'H',
  COOL_MED  = 'M',
  COOL_LOW  = 'L',
  STANDBY_HIGH = 'h',
  STANDBY_MED  = 'm',
  STANDBY_LOW  = 'l',
} ac_state_t;

typedef enum {
  FAN_OFF = '-',
  FAN_LOW = 'L',
  FAN_MED = 'M',
  FAN_HIGH = 'H'
} fan_state_t;

typedef enum {
  COMPRESSOR_OFF,
  COMPRESSOR_ON
} compressor_state_t;

// Governed by external input
volatile ac_cmd_t command = CMD_OFF;

// Internal states
volatile ac_state_t state = POWER_OFF;
volatile compressor_state_t compressorState = COMPRESSOR_OFF;
volatile fan_state_t fanState = FAN_OFF;

volatile long lastStateChangeTimeMs = millis();
volatile long startupTime = millis();
volatile long connectTime = millis();

void setup() {
  Serial.setDebugOutput(false);

  wifi_connect();

  printSensorAddresses();

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
          case 'O':  command = CMD_OFF;  break;
          case 'C':  command = CMD_COOL_HIGH; break;  
          case 'H':  command = CMD_COOL_HIGH; break;  
          case 'M':  command = CMD_COOL_MED; break;  
          case 'L':  command = CMD_COOL_LOW; break;  
          case 'K':  command = CMD_KILL; break;  
          case 'F':  command = CMD_FAN;  break; 
          default:   command = CMD_OFF;  
        }
      }
    }
    char* buffer = (char*)malloc(1024 * sizeof(char));
    sprintf(buffer, STATUS_JSON_FORMAT,
            TEMPERATURES_C[EVAP_IDX],
            TEMPERATURES_C[OUTLET_IDX],
            (millis() - startupTime),
            (millis() - connectTime),
            command,
            state,
            compressorState,
            fanState,
            getFreeHeap(),
            WiFi.status());
    server.send(200, "text/json", buffer);
    free(buffer);
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!is_wifi_connected()) {
    wifi_connect();
  }

  server.handleClient();
  MDNS.update();
  delay(100);
}

bool is_wifi_connected() {
  return WiFi.status() == WL_CONNECTED;
}

void wifi_connect() {
  // Must reset the wi-fi hardware each time it disconnects to ensure a reliable connection
  int attempts = 10;
  while (!is_wifi_connected() && (attempts--)) {
    Serial.print("Wi-fi status: ");
    Serial.println(WiFi.status());

    WiFi.disconnect(true);

    delay(10000);

    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.noLowPowerMode();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(10000);
  }

  if (!is_wifi_connected()) {
    Serial.print("Connect failed! Rebooting.");
    rp2040.reboot();
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

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

void setup1() {
  pinMode(COMPRESSOR_PIN, OUTPUT);
  digitalWrite(COMPRESSOR_PIN, LOW);

  pinMode(FAN_LOW_PIN, OUTPUT);
  digitalWrite(FAN_LOW_PIN, LOW);

  pinMode(FAN_MED_PIN, OUTPUT);
  digitalWrite(FAN_MED_PIN, LOW);

  pinMode(FAN_HIGH_PIN, OUTPUT);
  digitalWrite(FAN_HIGH_PIN, LOW);
}

void loop1() {
  readTemperatures(); 

  // Detect icing and stop cooling if detected. 
  if (!isEvaporatorInSafeTempRange())
  {
    setCompressor(COMPRESSOR_OFF); 
    Serial.println("Compressor in hold off due to unsafe temperatures."); 
    delay(20 * 1000); 
  }

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.printf("%s: %fC\r\n", SENSOR_NAMES[i], TEMPERATURES_C[i]);
  }

  state = processCommand(state, command);

  delay(1000);
}

ac_state_t changeState(const ac_state_t currentState, const ac_state_t targetState) {
  if (targetState != currentState) {
    Serial.printf("changeState: %c => %c\r\n", currentState, targetState);

    switch (targetState) {
      case COOL_HIGH:
        {
          setFan(FAN_HIGH);
          break;
        }
      case COOL_MED:
        {
          setFan(FAN_MED);
          break;
        }
      case COOL_LOW:
        {
          setFan(FAN_LOW);
          break;
        }
      case STANDBY_HIGH:
        {
          setCompressor(COMPRESSOR_OFF);
          setFan(FAN_HIGH);
          break;
        }
      case STANDBY_MED:
        {
          setCompressor(COMPRESSOR_OFF);
          setFan(FAN_MED);
          break;
        }
      case STANDBY_LOW:
        {
          setCompressor(COMPRESSOR_OFF);
          setFan(FAN_LOW);
          break;
        }
      case POWER_OFF:
        {
          setCompressor(COMPRESSOR_OFF);
          setFan(FAN_OFF);
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
    case COMPRESSOR_ON: return 
        (TEMPERATURES_C[EVAP_IDX] > 10.0 && TEMPERATURES_C[OUTLET_IDX] > 1.0)
        || TEMPERATURES_C[EVAP_IDX] > TEMPERATURES_C[OUTLET_IDX];
    case COMPRESSOR_OFF: return 
        TEMPERATURES_C[EVAP_IDX] > 10.0 && TEMPERATURES_C[OUTLET_IDX] > 1.0;
    default: return 1; 
  }
}

float calculateOnTemp(float stateOffTempC, float stateOnTempC, float minRangeC, float minEvapDeltaC, float maxEvapDeltaC)
{
  // If the state on temp is larger than the evap temp, we are unlikely to ever cool. Adjust it to just under. 
  float maxOnTempC = fmin(TEMPERATURES_C[EVAP_IDX] - minEvapDeltaC, stateOnTempC);

  // If we try to start with too large a delta, the compressor will see huge torque and fail to operate. 
  float minOnTempC = fmax(TEMPERATURES_C[EVAP_IDX] - maxEvapDeltaC, stateOffTempC + minRangeC); 

  if (maxOnTempC > minOnTempC)
  {
    return maxOnTempC; 
  }
  else 
  {
    return minOnTempC; 
  }
}

ac_state_t processCommand(const ac_state_t currentState, const ac_cmd_t command) {
  ac_state_t newState = currentState;
  switch (command) 
  {
    case CMD_COOL_HIGH:
    {
      float onTempC = calculateOnTemp(COOL_HIGH_OFF_TEMP_C, COOL_HIGH_ON_TEMP_C, 5.0, 5.0, 8.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case STANDBY_HIGH: newState = COOL_HIGH; break;
        case COOL_HIGH: controlCompressorState(COOL_HIGH_OFF_TEMP_C, onTempC); break;
        default: newState = STANDBY_HIGH; break;
      }
      break;
    }
    case CMD_COOL_MED:
    {
      float onTempC = calculateOnTemp(COOL_MED_OFF_TEMP_C, COOL_MED_ON_TEMP_C, 5.0, 2.0, 7.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case STANDBY_MED: newState = COOL_MED; break;
        case COOL_MED: controlCompressorState(COOL_MED_OFF_TEMP_C, onTempC); break;
        default: newState = STANDBY_MED; break;
      }
      break;
    }
    case CMD_COOL_LOW:
    {
      float onTempC = calculateOnTemp(COOL_LOW_OFF_TEMP_C, COOL_LOW_ON_TEMP_C, 5.0, 1.0, 5.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case STANDBY_LOW: newState = COOL_LOW; break;
        case COOL_LOW: controlCompressorState(COOL_LOW_OFF_TEMP_C, onTempC); break;
        default: newState = STANDBY_LOW; break;
      }
      break;
    }
    case CMD_OFF:
    {
      long elapsedStateTimeMs = millis() - lastStateChangeTimeMs;
      switch (currentState) 
      {
        case COOL_HIGH: newState = STANDBY_HIGH; break;
        case COOL_MED:  newState = STANDBY_MED;  break;
        case COOL_LOW:  newState = STANDBY_LOW;  break;
        case STANDBY_HIGH: newState = elapsedStateTimeMs > STANDBY_HIGH_TIME_MS ? STANDBY_MED : currentState; break;
        case STANDBY_MED:  newState = elapsedStateTimeMs > STANDBY_MED_TIME_MS  ? STANDBY_LOW : currentState; break;
        case STANDBY_LOW:  newState = elapsedStateTimeMs > STANDBY_LOW_TIME_MS  ? POWER_OFF   : currentState; break;
      }
      break;
    }
    case CMD_KILL: 
    {
      switch (currentState) 
      {
        case COOL_HIGH: newState = STANDBY_HIGH; break;
        case COOL_MED:  newState = STANDBY_MED;  break;
        case COOL_LOW:  newState = STANDBY_LOW;  break;
        default: newState = POWER_OFF; break;
      }
      break; 
    }
    case CMD_FAN: newState = STANDBY_HIGH; break;
  }

  if (newState != currentState) {
    Serial.printf("==> processCommand: %c: %d => %d\r\n", command, currentState, newState);
    newState = changeState(currentState, newState);
  }

  return newState;
}

void setFan(const fan_state_t targetState) {
  if (targetState != fanState) {
    // Turn off the currently energized winding, if any
    switch (fanState) {
      case FAN_LOW: digitalWrite(FAN_LOW_PIN, LOW); break;
      case FAN_MED: digitalWrite(FAN_MED_PIN, LOW); break;
      case FAN_HIGH: digitalWrite(FAN_HIGH_PIN, LOW); break;
    }

    // If transitioning from another on state, delay a brief time to allow the winding to de-energize.
    if (fanState != FAN_OFF && targetState != FAN_OFF)
      delay(FAN_SPEED_TRANSITION_TIME_MS);

    // Energize the traget winding, if any
    switch (targetState) {
      case FAN_LOW: digitalWrite(FAN_LOW_PIN, HIGH); break;
      case FAN_MED: digitalWrite(FAN_MED_PIN, HIGH); break;
      case FAN_HIGH: digitalWrite(FAN_HIGH_PIN, HIGH); break;
    }

    Serial.printf("setFan: %c => %c\r\n", fanState, targetState);
    delay(TRANSITION_TIME_MS);
    fanState = targetState;
  }
}

void setCompressor(const compressor_state_t targetState) {
  if (targetState != compressorState) {
    switch (targetState) {
      case COMPRESSOR_ON:  digitalWrite(COMPRESSOR_PIN, HIGH); break;
      case COMPRESSOR_OFF: digitalWrite(COMPRESSOR_PIN, LOW); break;
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
    case COMPRESSOR_ON: 
    {
      if (TEMPERATURES_C[OUTLET_IDX] <= offTempC)
        setCompressor(COMPRESSOR_OFF); 
      break; 
    }
    case COMPRESSOR_OFF: 
    {
      if (TEMPERATURES_C[OUTLET_IDX] >= onTempC)
        setCompressor(COMPRESSOR_ON); 
      break; 
    }
  }
}


// TEMP SENSORS

int isTempCValid(float tempC) 
{
  return tempC != INVALID_TEMP
    && tempC == tempC 
    && tempC < 40 
    && tempC > -40; 
}


int waitForTemperatures()
{
  Serial.println("Wait for sensors"); 
  Serial.println(" "); 
  char padding[] = "============================";
  int ready = 0; 
  for(int retry = 0; retry < 12 && !ready; retry++) 
  {
    delay(5000); 
    
    ready = 1; 
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (TEMPERATURES_C[i] == INVALID_TEMP)
        {
          Serial.printf("... %s\r\n", SENSOR_NAMES[i]); 
          ready = 0; 
          break;
        } 
        else 
        {
          Serial.printf("[OK] %s\r\n", SENSOR_NAMES[i]); 
        }
    }
  }

  if (ready)
  {
    Serial.printf("Sensors ready!\r\n");
  }
  else 
  {
    Serial.printf("Sensors not ready\r\n");
  }

  return ready; 
}

int countSensors() 
{
  int count = 0;
  while (ds.selectNext()) count++;
  return count;
}

int getSensorData(int* address, float* tempC, int nSensors) {
  int i = 0;
  while (ds.selectNext()) {
    if (i < nSensors) {
      uint8_t sensorAddress[8];
      ds.getAddress(sensorAddress);
      address[i] = sensorAddress[7];
      tempC[i] = ds.getTempC();
    }

    i++;
  }

  return i;
}

float getTempC(int requestedAddress, int* address, float* tempC, int nSensors)
{
  int i;
  for (i = 0; i < nSensors; i++) 
  {
    if (requestedAddress == address[i])
    {
      float tmpTempC = tempC[i];

      if (tmpTempC == tmpTempC)
      {
        return tmpTempC; 
      }
    }
  }

  return INVALID_TEMP;
}

void updateTemperature(int requestedAddress, int* address, float* tempC, int nSensors, volatile float* currentTempC)
{
  float newTempC = getTempC(requestedAddress, address, tempC, nSensors); 

  if (newTempC != INVALID_TEMP && newTempC == newTempC)
  {
    *currentTempC = newTempC; 
  }
}

void readTemperatures() 
{
  int nSensors = countSensors();
  int* address = (int*)malloc(nSensors * sizeof(int));
  float* tempC = (float*)malloc(nSensors * sizeof(float));
  nSensors = getSensorData(address, tempC, nSensors);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    int valid = 0; 
    float previousTempC = 0; 
    float newTempC = TEMPERATURES_C[i]; 
    do {
      previousTempC = newTempC; 
      updateTemperature(TEMP_SENSOR_ADDRESSES[i], address, tempC, nSensors, &(newTempC));
      int significantChange = newTempC != INVALID_TEMP && (int)(previousTempC * 1.0) != (int)(newTempC * 1.0); 
      valid = isTempCValid(newTempC) && !significantChange; 
      if (!valid)
      {
        Serial.printf("Invalid temperature change: %f to %f; resampling\r\n", previousTempC, newTempC); 
        delay(5000); 
        nSensors = getSensorData(address, tempC, nSensors);
      }
    } while(!valid); 
    TEMPERATURES_C[i] = newTempC; 
  }
  free(address);
  free(tempC);
}

/* Reads temperature sensors */
void printSensorAddresses()
{
  int nSensors = countSensors();
  if (nSensors == 0)
  {
    Serial.println("No sensors detected"); 
    return; 
  }

  int* address = (int*)malloc(nSensors * sizeof(int));
  float* tempC = (float*)malloc(nSensors * sizeof(float));
  char buffer[1024]; 
  nSensors = getSensorData(address, tempC, nSensors);
  buffer[0] = 0; 
  for (int i = 0; i < nSensors; i++)
  {
      sprintf(buffer + strlen(buffer), "%d ", address[i]); 
  }
  Serial.println(buffer); 
  free(address);
  free(tempC);
}

// Heap 

uint32_t getTotalHeap(void) {
  extern char __StackLimit, __bss_end__;

  return &__StackLimit - &__bss_end__;
}

uint32_t getFreeHeap(void) {
  struct mallinfo m = mallinfo();

  return getTotalHeap() - m.uordblks;
}

