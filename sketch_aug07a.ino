#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include "radio.h" 
#include "max6675.h"
#include <Wire.h>
#include <PID_v1.h>


#ifndef APSSID
#define APSSID "ESPap"
#define APPSK  "thereisnospoon"
#endif


int ktcSO = 12;
int ktcCS = 13;
int ktcCLK = 14;
 
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

String output;


const char *ssid = APSSID;
const char *password = APPSK;

ESP8266WebServer server(80);

/* This unbelievably shabby code is meant to roast coffee :)
 *  It's intended to control a remote-control mains switch via a 433Mhz transmitter.
 *  It's loosely based on this Jaycar project:
 *  https://www.jaycar.com.au/wifi-mains-switch
 *  It also talks to a thermocouple via a MAX6675 thermocouple amp.
 *  The idea is to plug a popcorn popper into the remote-control mains switch, then switch it on and
 *  off using the transmitter. You specify a target temperature; when the thermocouple detects it twice
 *  in a row, it switches the whole thing off.
 *  It also logs timestamped temperature measurements out to the serial port.
 *  So it's pretty basic:
 *    switch on for x
 *    measure temp, record max
 *    switch off for x
 *    measure temp, record max
 *    
 * The appeal of this is that I can roast in a fairly repeatable way using common household items while
 * not messing with mains voltage (so no need to modify the popper, for example).
 * 
 * What would obvs be better from a repeatability p.o.v. would be to do some sort of PID type thing.
 * I've done the reading - this looks feasible. The appeal here is that this *should* be more or less
 * indifferent to ambient air temperature.
 * From the looks of things, a 30 second sampling interval is a decent compromise, bearing in mind that
 * each sampling cycle involves 1 cycle on the relay. Assuming 140 cycles to roast a batch (of about 220g),
 * and a working life of 100,000 cycles, that's 700 roasts, or about 150kg of coffee. So that's probably OK.
 * The switch is a whole $9.95, so that's, like, replacing a $10 part after 2 years of daily use. Meh.
 *
 * Targeting a "LOLIN WeMos D1 R2 & Mini" works OK. Targeting a generic 8266 does not.
 */

 struct setpoint_record {
  int timestamp;
  int setpoint;
};

std::vector<setpoint_record> setpoint_records_general = {
  {60*1000, 120},
  {120 *1000,120},
  {180 *1000,140},
  {240 *1000,140},
  {300 *1000,160},
  {360 *1000,160},
  {420 *1000,180},
  {480 *1000,180},
  {540 *1000,200},
  {600 *1000,200},
  {660 *1000,205},
  {720 *1000,205},
  {780 *1000,210},
  {840 *1000,215},
  {900 *1000,220}};

std::vector<setpoint_record> setpoint_records_chesterton = {
  {60*1000, 120},
  {120 *1000,120},
  {180 *1000,140},
  {240 *1000,140},
  {300 *1000,160},
  {360 *1000,160},
  {420 *1000,180},
  {480 *1000,180},
  {540 *1000,200},
  {600 *1000,200},
  {660 *1000,205},
  {720 *1000,205},
  {780 *1000,210},
  {840 *1000,215}};

void setup()
{
  Serial.begin(9600);
  pinMode(D4, OUTPUT);
  pinMode(D3, OUTPUT);
}

double logStateAndReturnTemperature(int time_offset, int setpoint, bool state) {
  int current_time = millis() - time_offset;
  Serial.print(setpoint);
  output += setpoint;
  Serial.print(",");
  output += ",";
  Serial.print(current_time);
  output += current_time;
  Serial.print(",");
  output += ",";
  double temp = ktc.readCelsius(); 
  Serial.print(temp);
  output += temp;
  if(state) {
    Serial.print(",ON\r\n");
    output += ",ON\r\n";
  }
  else {
    Serial.print(",OFF\r\n");
    output += ",OFF\r\n";
  }
  return temp;
}

void non_pid_controller() {
  int start = millis();
  int on_interval = 10000;
  int off_interval = 20000;
  int max_temp = 0;
  int temp = 0;
  int target_temp = 220;
  int iterations = 0;
  int max_iterations = 40;

  // exit criteria: target_temp exceeded twice OR max iterations exceeded
  while(temp < target_temp && max_temp < target_temp && iterations <= max_iterations) {
        
    max_temp = max(temp, max_temp);
    radioSwitch(D4, 1, true);
    temp = logStateAndReturnTemperature(start, 0, "ON");
    delay(on_interval);
        
    radioSwitch(D4, 1, false); //sw1 OFF
    temp = logStateAndReturnTemperature(start, 0, "OFF");
    delay(off_interval);
    iterations++;
  }
  while(true)  radioSwitch(D4, 1, false); //sw1 OFF
}

// latest attempt at a roast profile. Big differences:
// no more than 10 degrees per minute temp change (the PID controller doesn't seem to like
// big changes - no surprise there
// tail temp rise off after first crack (from 660 onwards)
// Seems to work pretty well, except for the Chesterton A (see below)
double calculate_setpoint_general(int time) {
  if(time < 60  *1000) return 120;
  if(time < 120 *1000) return 120;
  if(time < 180 *1000) return 140;
  if(time < 240 *1000) return 140;
  if(time < 300 *1000) return 160;
  if(time < 360 *1000) return 160;
  if(time < 420 *1000) return 180;
  if(time < 480 *1000) return 180;
  if(time < 540 *1000) return 200;
  if(time < 600 *1000) return 200;
  if(time < 660 *1000) return 205;
  if(time < 720 *1000) return 205;
  if(time < 780 *1000) return 210;
  if(time < 840 *1000) return 215;  
  if(time < 900 *1000) return 220;  
  return 223;
}


double calculate_setpoint(int time, std::vector<setpoint_record> setpoint_records, int target_temp) {  

  for(const setpoint_record potential_setpoint: setpoint_records)
    if(time <= potential_setpoint.timestamp) 
      return potential_setpoint.setpoint;
  return target_temp;
}

void pid_controller() {
  // based on https://github.com/br3ttb/Arduino-PID-Library/blob/master/examples/PID_RelayOutput/PID_RelayOutput.ino
  double setpoint, input, output;
  int target_temp = 216;

  digitalWrite(D3, HIGH);
  //Specify the links and initial tuning parameters
  double kp=5, ki=3, kd=1;
  PID my_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

  int window_size = 20000;
  int window_start_time = millis();

  //initialize the variables we're linked to
  setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_chesterton, target_temp);

  //tell the PID to range between 0 and the full window size
  my_pid.SetOutputLimits(0, window_size);

  //turn the PID on
  my_pid.SetMode(AUTOMATIC);
  input = logStateAndReturnTemperature(0, setpoint, "OFF");
  setup_webserver();

  // bail when we hit target temp, or have been running for 20 minutes.
  while(input <= target_temp && millis() <= 20*60*1000) {
    setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_chesterton, target_temp);
    my_pid.Compute();

    bool state = output > setpoint;
    radioSwitch(D4, 1, state);
    input = logStateAndReturnTemperature(0, setpoint, state);
    server.handleClient();
  }
  Serial.print("Complete\n");
  digitalWrite(D3, LOW);
  while(true) {
    radioSwitch(D4, 1, false); //make sure it finishes in an off state
    server.handleClient();
  }
}

void handleRoot() {
  server.sendHeader("Content-Disposition", "attachment; filename=roasting_temps.csv");
  server.send(200, "text/csv", output);
}

void setup_webserver() {
  delay(1000);
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  pid_controller();
}
