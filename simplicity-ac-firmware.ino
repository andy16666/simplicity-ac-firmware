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
#include "config.h"

// Active high outputs 
#define COMPRESSOR_PIN 10
#define FAN_LOW_PIN 11
#define FAN_MED_PIN 12
#define FAN_HIGH_PIN 13

// Fan will satge down after turning off the unit from standby high -> med -> low -> off
// This allows the longer ducts I'm using to warm back up to room temperature and the coil 
// to dry after a cooling phase. It also prevents the fan from cycling on and off which 
// can be annoying at night. 1, 5 and 15 minutes for high, medium and then low keeps 
// the fan running long enough that if cooling is needed again, it will probably not 
// have stopped yet. 
#define STANDBY_HIGH_TIME_MS  60000
#define STANDBY_MED_TIME_MS  300000
#define STANDBY_LOW_TIME_MS  900000

// Transition time for changes to fan speed and compressor state. This must be long enough 
// for the circuit breaker to recover. 1s is much too short. 5s is close to the factory 
// setting. 
#define TRANSITION_TIME_MS     5000

// The fan uses 3 different windings, one for each speed. This delay allows the previous 
// winding to de-energize before engaging the new one. Must be long enough to allow the 
// magnetic field to dissipate but not so long that the fan stops. 100ms is a good starting 
// point. 
#define FAN_SPEED_TRANSITION_TIME_MS 100 

WebServer server(80);

// External command to the unit
typedef enum {
  OFF = '-', 
  COOL = 'C', 
  FAN = 'F'
} ac_cmd_t;

// A/C state is changed based on the external command and the current state. 
typedef enum {
  POWER_OFF    = '-', 
  COOL_HIGH    = 'C', 
  STANDBY_HIGH = 'H', 
  STANDBY_MED  = 'M', 
  STANDBY_LOW  = 'L',
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
volatile ac_cmd_t command = OFF;

// Internal states
volatile ac_state_t state = POWER_OFF; 
volatile compressor_state_t compressorState = COMPRESSOR_OFF; 
volatile fan_state_t fanState = FAN_OFF; 

volatile long lastStateChangeTimeMs = millis(); 

void setup() {
  Serial.setDebugOutput(false);

  wifi_connect();  

  server.begin(); 

  if (MDNS.begin(HOSTNAME)) {
    Serial.println("MDNS responder started");
  }

  server.on("/", []() {
    for (uint8_t i = 0; i < server.args(); i++) {
      String argName = server.argName(i); 
      String arg = server.arg(i); 

      if (argName.equals("status")) {
        command = arg.equals("on") ? COOL : OFF;
        Serial.print("Changed status to "); 
        Serial.println(((command == COOL) ? "COOL" : "OFF"));
      }

      if (argName.equals("fan")) {
        command = arg.equals("on") ? FAN : OFF;
        Serial.print("Changed status to "); 
        Serial.println(((command == FAN) ? "FAN" : "OFF"));
      }

      if (argName.equals("cool")) {
        command = arg.equals("on") ? COOL : OFF;
        Serial.print("Changed status to "); 
        Serial.println(((command == COOL) ? "COOL" : "OFF"));
      }
    }
    String statusJson = "{\n";
    statusJson += "\"status\":";
    statusJson += (command == OFF ? "0" : "1"); 
    statusJson += "\n}";
    server.send(200, "text/json", statusJson);
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
  while(!is_wifi_connected()) {
    Serial.print("Wi-fi status: ");
    Serial.println(WiFi.status());

    WiFi.disconnect(true); 

    delay(1000);

    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA); 
    WiFi.noLowPowerMode();
    WiFi.begin(WIFI_SSID, WIFI_PASS); 
    delay(10000);
  }

  Serial.println("Connected");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Hostname: "); 
  Serial.print(HOSTNAME);
  Serial.println(".local");  
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
  state = processCommand(state, command); 
  delay(1000); 
}

ac_state_t changeState(const ac_state_t currentState, const ac_state_t targetState) {
  if (targetState != currentState)
  {
    Serial.printf("changeState: %c => %c\r\n", currentState, targetState);

    switch(targetState) {
      case COOL_HIGH: {
        setFan(FAN_HIGH);
        setCompressor(COMPRESSOR_ON); 
        break; 
      }
      case STANDBY_HIGH: {
        setCompressor(COMPRESSOR_OFF); 
        setFan(FAN_HIGH);
        break; 
      }
      case STANDBY_MED: {
        setCompressor(COMPRESSOR_OFF); 
        setFan(FAN_MED);
        break; 
      }
      case STANDBY_LOW: {
        setCompressor(COMPRESSOR_OFF); 
        setFan(FAN_LOW);
        break; 
      }
      case POWER_OFF: {
        setCompressor(COMPRESSOR_OFF); 
        setFan(FAN_OFF);
        break; 
      }
    }
    lastStateChangeTimeMs = millis(); 
  }

  return targetState; 
}

ac_state_t processCommand(const ac_state_t currentState, const ac_cmd_t command) {
  ac_state_t newState = currentState; 
  switch(command) {
    case COOL: {
      // Move to STANDBY_HIGH, then to COOL_HIGH. 
      switch(currentState) {
          case STANDBY_HIGH: newState = COOL_HIGH;    break; 
          case COOL_HIGH:    break; 
          default:           newState = STANDBY_HIGH; break; 
      }
      break; 
    }
    case OFF: {
      long elapsedStateTimeMs = millis() - lastStateChangeTimeMs; 
      switch(currentState) {
        case COOL_HIGH:    newState = STANDBY_HIGH; break; 
        case STANDBY_HIGH: newState = elapsedStateTimeMs > STANDBY_HIGH_TIME_MS ? STANDBY_MED : currentState; break; 
        case STANDBY_MED:  newState = elapsedStateTimeMs > STANDBY_MED_TIME_MS  ? STANDBY_LOW : currentState; break; 
        case STANDBY_LOW:  newState = elapsedStateTimeMs > STANDBY_LOW_TIME_MS  ? POWER_OFF   : currentState; break; 
      }
      break; 
    }
    case FAN: newState = STANDBY_HIGH; break; 
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
    switch(fanState) {
      case FAN_LOW:  digitalWrite(FAN_LOW_PIN, LOW);  break; 
      case FAN_MED:  digitalWrite(FAN_MED_PIN, LOW);  break; 
      case FAN_HIGH: digitalWrite(FAN_HIGH_PIN, LOW); break; 
    }

    // If transitioning from another on state, delay a brief time to allow the winding to de-energize. 
    if (fanState != FAN_OFF && targetState != FAN_OFF)
      delay(FAN_SPEED_TRANSITION_TIME_MS); 
  
    // Energize the traget winding, if any 
    switch(targetState) { 
      case FAN_LOW:  digitalWrite(FAN_LOW_PIN, HIGH);  break; 
      case FAN_MED:  digitalWrite(FAN_MED_PIN, HIGH);  break; 
      case FAN_HIGH: digitalWrite(FAN_HIGH_PIN, HIGH); break; 
    }

    Serial.printf("setFan: %c => %c\r\n", fanState, targetState);
    delay(TRANSITION_TIME_MS);
    fanState = targetState; 
  }
}

void setCompressor(const compressor_state_t targetState) {
  if (targetState != compressorState) {
    switch(targetState) {
      case COMPRESSOR_ON:  digitalWrite(COMPRESSOR_PIN, HIGH); break; 
      case COMPRESSOR_OFF: digitalWrite(COMPRESSOR_PIN, LOW);  break; 
    }

    Serial.printf("setCompressor: %d => %d\r\n", compressorState, targetState);
    delay(TRANSITION_TIME_MS); 
    compressorState = targetState; 
  }
}


