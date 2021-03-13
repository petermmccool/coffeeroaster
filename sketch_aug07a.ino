#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "radio.h" 
#include "max6675.h"
#include <Wire.h>
#include <PID_v1.h>
#include "config.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define statusLEDPin 32
#define popperPowerSwitch 33

int thermocoupleSO = 19;
int thermocoupleCS = 23;
int thermocoupleCLK = 5;
 
MAX6675 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);
Adafruit_BME280 ambient_temperature_sensor;

// No garbage collection, grumble grumble
String output;
String message;

// WiFi/webserver stuff
const char *ssid = APSSID;
const char *password = APPSK;

WebServer server(80);

/* This unbelievably shabby code is meant to roast coffee :)
 *  It's intended to control a remote-control mains switch via a 433Mhz transmitter.
 *  It's loosely based on this Jaycar project:
 *  https://www.jaycar.com.au/wifi-mains-switch
 *  It also listens to a thermocouple via a MAX6675 thermocouple amp, which is on the SPI bus
 *  The idea is to plug a popcorn popper into the remote-control mains switch, then switch it on and
 *  off using the transmitter. You specify a target temperature; when the thermocouple detects it twice
 *  in a row, it switches the whole thing off.
 *  It also logs timestamped temperature measurements out to the serial port. Format:
 *  setpoint,current_time,thermocouple_reading,ambient_temperature,humidity,relay_state.
 *  Temperatures are in Celsius.
 *  It uses a PID controller to decide when to switch the popper on/off
 *  ESP32 pinout: https://www.instructables.com/ESP32-Internal-Details-and-Pinout/
 *    
 */

 struct setpoint_record {
  int timestamp;
  int setpoint;
};

std::vector<setpoint_record> setpoint_records_general2 = {
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

// started life as a profile for the Chesterton, but I find I prefer the results :)
  std::vector<setpoint_record> setpoint_records_general3 = {
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

std::vector<setpoint_record> setpoint_records_general = {
  {60*1000, 100},
  {120 *1000,110},
  {180 *1000,120},
  {240 *1000,130},
  {300 *1000,140},
  {360 *1000,150},
  {420 *1000,160},
  {480 *1000,170},
  {540 *1000,180},
  {600 *1000,190},
  {660 *1000,200},
  {720 *1000,207},
  {780 *1000,214},
  {840 *1000,220}};


std::vector<setpoint_record> setpoint_records_slow = {
  {60*1000, 100},
  {120 *1000,110},
  {180 *1000,120},
  {240 *1000,130},
  {300 *1000,140},
  {360 *1000,150},
  {420 *1000,160},
  {480 *1000,170},
  {540 *1000,180},
  {600 *1000,190},
  {660 *1000,195},
  {720 *1000,200},
  {780 *1000,205},
  {840 *1000,210},
  {900 *1000,215},
  {900 *1000,220}};

std::vector<setpoint_record> setpoint_records_chesterton2 = {
  {60*1000, 120},
  {120 *1000,120},
  {180 *1000,140},
  {240 *1000,140},
  {300 *1000,160},
  {360 *1000,160},
  {420 *1000,180},
  {480 *1000,180},
  {540 *1000,190},
  {600 *1000,200},
  {660 *1000,200},
  {720 *1000,205},
  {780 *1000,205},
  {840 *1000,210}};

void setup()
{
  Serial.begin(9600);
  pinMode(popperPowerSwitch, OUTPUT);
  pinMode(statusLEDPin, OUTPUT);
  bool status = ambient_temperature_sensor.begin(0x77); // I think? or 0x76?
  if (!status) {
    Serial.println("Could not find ambient temperature sensor");
  }
  

  setup_webserver();
}

double logStateAndReturnTemperature(int time_offset, int setpoint, bool state) {
  int current_time = millis() - time_offset;
  message = setpoint;
  message += ",";
  message += current_time;
  message += ",";
  double temp = thermocouple.readCelsius(); 
  message += temp;
  double ambient_temp = ambient_temperature_sensor.readTemperature();
  message += ",";
  message += ambient_temp;
  double humidity = ambient_temperature_sensor.readHumidity();
  message += ",";
  message += humidity;
  if(state) 
    message += ",ON\r\n";
  else
    message += ",OFF\r\n";

  Serial.print(message);
  output += message;

  return temp;
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
  //int target_temp = 208;
  int target_temp = 203; // latest attempt at making Chesterton not taste of ash :)
  int start_temp = 120;

  // The point here is to get up to operating temperature ASAP. The thinking here is that the PID controller will behave more
  // predictably if the temp is roughtly at the first setpoint when it starts. So just run the popper flat-out till it gets there :)
  // the counter is just a really cheap & nasty way of making sure the popper doesn't stay on indefinitely
  digitalWrite(statusLEDPin, HIGH);
  int count = 0;
  Serial.print("Warming up to " + start_temp + "\n");
  while(thermocouple.readCelsius() < start_temp && count < 20) {
    radioSwitch(popperPowerSwitch, 1, true);
    count++;
  }

  radioSwitch(popperPowerSwitch, 1, false);
 
  double kp=5, ki=3, kd=1;
  PID my_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
  int window_size = 20000;
  int window_start_time = millis();

  //initialize the variables we're linked to
  setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_slow, target_temp);

  //tell the PID to range between 0 and the full window size
  my_pid.SetOutputLimits(0, window_size);

  //turn the PID on
  my_pid.SetMode(AUTOMATIC);
  input = logStateAndReturnTemperature(0, setpoint, "OFF");

  // bail when we hit target temp, or have been running for 20 minutes.
  while(input <= target_temp && millis() <= 20*60*1000) {
    setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_slow, target_temp);
    my_pid.Compute();

    bool state = output > setpoint;
    radioSwitch(popperPowerSwitch, 1, state);
    input = logStateAndReturnTemperature(0, setpoint, state);
    server.handleClient();
  }
  Serial.print("Complete\n");
  digitalWrite(statusLEDPin, LOW);
  while(true) {
    radioSwitch(popperPowerSwitch, 1, false); //make sure it finishes in an off state
    server.handleClient();
  }
}

void handleRoot() {
  server.sendHeader("Content-Disposition", "attachment; filename=roasting_temps.csv");
  server.send(200, "text/csv", output);
}

void setup_webserver() {
  delay(1000);
  Serial.print("Configuring access point on");
  Serial.print(ssid);
  Serial.print(password);
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
