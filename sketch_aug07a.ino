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

#define D3 32
#define D4 33

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
  {840 *1000,215}};

std::vector<setpoint_record> setpoint_records_chesterton = {
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
  pinMode(D4, OUTPUT);
  pinMode(D3, OUTPUT);
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
  int target_temp = 216;

  digitalWrite(D3, HIGH);
  //Specify the links and initial tuning parameters
  double kp=5, ki=3, kd=1;
  PID my_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

  int window_size = 20000;
  int window_start_time = millis();

  //initialize the variables we're linked to
  setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_general, target_temp);

  //tell the PID to range between 0 and the full window size
  my_pid.SetOutputLimits(0, window_size);

  //turn the PID on
  my_pid.SetMode(AUTOMATIC);
  input = logStateAndReturnTemperature(0, setpoint, "OFF");

  // bail when we hit target temp, or have been running for 20 minutes.
  while(input <= target_temp && millis() <= 20*60*1000) {
    setpoint = calculate_setpoint(millis() - window_start_time, setpoint_records_general, target_temp);
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

  IPAddress myIP = WiFi.localIP();
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
