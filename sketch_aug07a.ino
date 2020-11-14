#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "radio.h" 
#include "max6675.h"
#include <Wire.h>
#include <PID_v1.h>
#include "config.h"
#include <AzureIoTHub.h>
#include "AzureIoTProtocol_MQTT.h"
#include "iothubtransportmqtt.h"

#define D3 32
#define D4 33

int ktcSO = 19;
int ktcCS = 23;
int ktcCLK = 5;
=======
 
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

// IoT stuff
#define USE_BALTIMORE_CERT
IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle;
IOTHUB_MESSAGE_HANDLE message_handle;
IOTHUB_CLIENT_RESULT result;
static const char* connectionString = DEVICE_CONNECTION_STRING;

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
 *  It also listens to a thermocouple via a MAX6675 thermocouple amp.
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

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
  (void)reason;
  (void)user_context;
  // This sample DOES NOT take into consideration network outages.
  if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    Serial.print("Connection status: The device client is connected to iothub\r\n");
  else {
    Serial.print("Connection status: The device client has been disconnected\r\n");
    Serial.print(IOTHUB_CLIENT_CONNECTION_STATUS_REASONStrings(reason);
  }
}

static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    Serial.print("Confirmation callback received for message %lu with result ");
    Serial.println(IOTHUB_CLIENT_CONFIRMATION_RESULTStrings(result));
}

void setup()
{
  Serial.begin(9600);
  pinMode(D4, OUTPUT);
  pinMode(D3, OUTPUT);

  setup_webserver();

  IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;

  (void)IoTHub_Init();
  device_ll_handle = IoTHubDeviceClient_LL_CreateFromConnectionString(connectionString, protocol);
  Serial.print("Creating IoTHub Device handle\r\n");

  if (device_ll_handle == NULL)
  {
      Serial.print("Error AZ002: Failure creating Iothub device. Hint: Check you connection string.\r\n");
  }
  else
  {
    // Setting the Trusted Certificate.
    IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_TRUSTED_CERT, certificates);

    bool urlEncodeOn = true;
    IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_AUTO_URL_ENCODE_DECODE, &urlEncodeOn);

    // Setting connection status callback to get indication of connection to iothub
    (void)IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_ll_handle, connection_status_callback, NULL);
    IoTHubDeviceClient_LL_DoWork(device_ll_handle);
    Serial.println("set connection status callback");
  }
}

double logStateAndReturnTemperature(int time_offset, int setpoint, bool state) {
  int current_time = millis() - time_offset;
  message = setpoint;
  message += ",";
  message += current_time;
  message += ",";
  double temp = ktc.readCelsius(); 
  message += temp;
  if(state) 
    message += ",ON\r\n";
  else
    message += ",OFF\r\n";

  Serial.print(message);
  output += message;

  message_handle = IoTHubMessage_CreateFromString(message.c_str());

  Serial.print("Sending message %d to IoTHub\r\n");
  result = IoTHubDeviceClient_LL_SendEventAsync(device_ll_handle, message_handle, send_confirm_callback, NULL);
  // should probs do something with result...
  IoTHubMessage_Destroy(message_handle);
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
    IoTHubDeviceClient_LL_DoWork(device_ll_handle);

    bool state = output > setpoint;
    radioSwitch(D4, 1, state);
    input = logStateAndReturnTemperature(0, setpoint, state);
    IoTHubDeviceClient_LL_DoWork(device_ll_handle);
    server.handleClient();
    IoTHubDeviceClient_LL_DoWork(device_ll_handle);
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
  //WiFi.softAP(ssid, password);
  WiFi.begin(ssid, password);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

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
