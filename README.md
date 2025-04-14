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

