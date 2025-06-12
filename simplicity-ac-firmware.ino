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

#define HOSTNAME "ac1"
#define AC_HOSTNAME HOSTNAME

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <CPU.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include <aos.h>
#include <config.h>
#include <GPIOOutputs.h>
#include <SimplicityAC.h> 

using namespace AOS; 
using AOS::GPIOOutputs; 


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

SimplicityAC AC(HOSTNAME);
GPIOOutputs OUTPUTS("AC Outputs")

// Internal states
volatile ac_state_t state = AC_POWER_OFF;
volatile compressor_state_t compressorState = AC_COMPRESSOR_OFF;
volatile fan_state_t fanState = AC_FAN_OFF;

static volatile ac_cmd_t        command                __attribute__((section(".uninitialized_data")));

volatile long lastStateChangeTimeMs = millis();

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
  command = CMD_AC_OFF;
}

void aosSetup() 
{
  
}

void aosSetup1() 
{
  TEMPERATURES.add("Evap", "evapTempC", EVAP_TEMP_ADDR); 
  TEMPERATURES.add("Outlet", "outletTempC", OUTLET_TEMP_ADDR); 

  OUTPUTS.add("Compressor", COMPRESSOR_PIN, true); 
  OUTPUTS.add("Fan Low", FAN_LOW_PIN, true);
  OUTPUTS.add("Fan Medium", FAN_MED_PIN, true);
  OUTPUTS.add("Fan High", FAN_HIGH_PIN, true);
  OUTPUTS.init(); 
  OUTPUTS.setAll(); 

  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_processCommands);  
}

void populateHttpResponse(JsonDocument& document) 
{
  document["command"] = String((char)command).c_str(); 
  document["state"] = String((char)state).c_str(); 
  document["fanState"] = String((char)fanState).c_str(); 
  document["compressorState"] = String((int)compressorState).c_str(); 
}

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  if (argName.equals("cmd") && arg.length() == 1) 
  {
    switch(arg.charAt(0))
    {
      case '-':  command = CMD_AC_OFF;  break;
      case 'O':  command = CMD_AC_OFF;  break;
      case 'C':  command = CMD_AC_COOL_HIGH; break;  
      case 'H':  command = CMD_AC_COOL_HIGH; break;  
      case 'M':  command = CMD_AC_COOL_MED; break;  
      case 'L':  command = CMD_AC_COOL_LOW; break;  
      case 'K':  command = CMD_AC_KILL; break;  
      case 'F':  command = CMD_AC_FAN_HIGH;  break; 
      case 'm':  command = CMD_AC_FAN_MED;  break; 
      case 'l':  command = CMD_AC_FAN_LOW;  break; 
      default:   success = false;  
    }
  }

  return success; 
}

void task_processCommands()
{
  state = processCommand(state, command);
}

ac_state_t changeState(const ac_state_t currentState, const ac_state_t targetState) 
{
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
      case AC_STATE_FAN_HIGH:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_HIGH);
          break;
        }
      case AC_STATE_FAN_MED:
        {
          setCompressor(AC_COMPRESSOR_OFF);
          setFan(AC_FAN_MED);
          break;
        }
      case AC_STATE_FAN_LOW:
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
        TEMPERATURES.getTempC(EVAP_TEMP_ADDR) > 0.5 && TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) > 0.5;
    case AC_COMPRESSOR_OFF: return 
        TEMPERATURES.getTempC(EVAP_TEMP_ADDR) > 5.0 && TEMPERATURES.getTempC(OUTLET_TEMP_ADDR) > 5.0;
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
        case AC_STATE_FAN_HIGH: newState = AC_COOL_HIGH; break;
        case AC_COOL_HIGH: controlCompressorState(COOL_HIGH_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STATE_FAN_HIGH; break;
      }
      break;
    }
    case CMD_AC_COOL_MED:
    {
      float onTempC = calculateOnTemp(COOL_MED_OFF_TEMP_C, COOL_MED_ON_TEMP_C, 5.0, 2.0, 7.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case AC_STATE_FAN_MED: newState = AC_COOL_MED; break;
        case AC_COOL_MED: controlCompressorState(COOL_MED_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STATE_FAN_MED; break;
      }
      break;
    }
    case CMD_AC_COOL_LOW:
    {
      float onTempC = calculateOnTemp(COOL_LOW_OFF_TEMP_C, COOL_LOW_ON_TEMP_C, 5.0, 1.0, 5.0);

      // Move to STANDBY_HIGH, then to COOL_HIGH.
      switch (currentState) 
      {
        case AC_STATE_FAN_LOW: newState = AC_COOL_LOW; break;
        case AC_COOL_LOW: controlCompressorState(COOL_LOW_OFF_TEMP_C, onTempC); break;
        default: newState = AC_STATE_FAN_LOW; break;
      }
      break;
    }
    case CMD_AC_OFF:
    {
      long elapsedStateTimeMs = millis() - lastStateChangeTimeMs;
      switch (currentState) 
      {
        case AC_COOL_HIGH: newState = AC_STATE_FAN_HIGH; break;
        case AC_COOL_MED:  newState = AC_STATE_FAN_MED;  break;
        case AC_COOL_LOW:  newState = AC_STATE_FAN_LOW;  break;
        case AC_STATE_FAN_HIGH: newState = elapsedStateTimeMs > STANDBY_HIGH_TIME_MS ? AC_STATE_FAN_MED : currentState; break;
        case AC_STATE_FAN_MED:  newState = elapsedStateTimeMs > STANDBY_MED_TIME_MS  ? AC_STATE_FAN_LOW : currentState; break;
        case AC_STATE_FAN_LOW:  newState = elapsedStateTimeMs > STANDBY_LOW_TIME_MS  ? AC_POWER_OFF     : currentState; break;
      }
      break;
    }
    case CMD_AC_KILL: 
    {
      switch (currentState) 
      {
        case AC_COOL_HIGH: newState = AC_STATE_FAN_HIGH; break;
        case AC_COOL_MED:  newState = AC_STATE_FAN_MED;  break;
        case AC_COOL_LOW:  newState = AC_STATE_FAN_LOW;  break;
        default: newState = AC_POWER_OFF; break;
      }
      break; 
    }
    case CMD_AC_FAN_HIGH: newState = AC_STATE_FAN_HIGH; break;
    case CMD_AC_FAN_MED:  newState = AC_STATE_FAN_MED;  break;
    case CMD_AC_FAN_LOW:  newState = AC_STATE_FAN_LOW;  break;
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
      case AC_FAN_LOW:  OUTPUTS.setCommand(FAN_LOW_PIN, LOW); break;
      case AC_FAN_MED:  OUTPUTS.setCommand(FAN_MED_PIN, LOW); break;
      case AC_FAN_HIGH: OUTPUTS.setCommand(FAN_HIGH_PIN, LOW); break;
    }

    OUTPUTS.execute(); 

    // If transitioning from another on state, delay a brief time to allow the winding to de-energize.
    if (fanState != AC_FAN_OFF && targetState != AC_FAN_OFF)
      delay(FAN_SPEED_TRANSITION_TIME_MS);

    // Energize the traget winding, if any
    switch (targetState) 
    {
      case AC_FAN_LOW: OUTPUTS.setCommand(FAN_LOW_PIN, HIGH); break;
      case AC_FAN_MED: OUTPUTS.setCommand(FAN_MED_PIN, HIGH); break;
      case AC_FAN_HIGH: OUTPUTS.setCommand(FAN_HIGH_PIN, HIGH); break;
    }

    OUTPUTS.execute(); 

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
      case AC_COMPRESSOR_ON:  OUTPUTS.setCommand(COMPRESSOR_PIN, HIGH); break;
      case AC_COMPRESSOR_OFF: OUTPUTS.setCommand(COMPRESSOR_PIN, LOW); break;
    }

    OUTPUTS.execute(); 

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
