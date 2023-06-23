// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
#include <Arduino.h>
#include <ESPUI.h>
#include <main.h>

#if defined(ESP32)
#include <ESPmDNS.h>
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif

#include <ArduinoOTA.h>
#include <DHTesp.h>
#include <NTPClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <neotimer.h>

// Settings
#define SLOW_BOOT 0
#define FORCE_USE_HOTSPOT 0

#define DHTTYPE DHT22
#define HOSTNAME "thermostat"

#define DHT_PIN 7
#define RELAY_PIN 8

const char *mqtt_server = MQTT_SERVER;
const char *mqtt_topic = MQTT_TOPIC;

const unsigned long maxScen = 600000;  // 10 minutes

// UI handles
uint16_t wifi_ssid_text, wifi_pass_text;
volatile bool updates = false;

// Heating system
uint16_t heatingSwitch = 0;
uint16_t startHourWeekday = 0;
uint16_t endHourWeekday = 0;

uint16_t startHourWeekend = 0;
uint16_t endHourWeekend = 0;

uint16_t thermostatTemperature = 0;
float currentTemperature = 0;
uint16_t runlevelScenario = 0;
unsigned long startScen = 0;   // scenario started
uint16_t boilerStat = 0;       // boiler status
unsigned long scenLength = 0;  // minutes on

Preferences preferences;

WiFiUDP ntpUDP;
DHTesp dht;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 3600000);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

Neotimer sensorsTimer = Neotimer(10000);

void textCallback(Control *sender, int type);
void generalCallback(Control *sender, int type);
void enterWifiDetailsCallback(Control *sender, int type);
void randomString(char *buf, int len);

void connectWifi();
void connectMqtt();
void setUpUI();
void loadControlValues();
void saveControlValues();
void updateScreen();
void sendSensor();
void loop();
void runtimeScenarios();
void printDebugInformation();

boolean areConditionsForRunlevel1Passed();
boolean isWeekend();
boolean isScheduleOn(int startHour, int endHour);
boolean isScheduleOn(boolean isWeekend);

void logMessage(String message, float toshow);
void logMessage(String message, float toshow);
void logMessage(String message);
void logMessage(String message, int toshow);
void logMessage(String message, boolean toshow);
void logMessage(String message, String toshow);
void logMessage(String message, float toshow);
void logMessage(String message);
void logInDisplay(String message);

unsigned long boilerOnTime(int actTemp, int setTemp);
int checkOnBoiler(unsigned long starts, unsigned long expectedTimeOn,
                  unsigned long currentMillisOn, int currentTemperature,
                  int themorstatExpectedTemperature);
int gotoRunlevel1(float actualTemperature, int thermostatTemperature);
String hour2str(int hour);

void setup() {
  preferences.begin("my-app", false);
  //   setCpuFrequencyMhz(160);
  randomSeed(0);
  Serial.begin(115200);
  while (!Serial)
    ;

  connectWifi();
#if defined(ESP32)
  WiFi.setSleep(false);  // For the ESP32: turn off sleeping to increase UI
                         // responsivness (at the cost of power use)
#endif
  connectMqtt();
  setUpUI();

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  mqttClient.setServer(mqtt_server, 1883);
  Serial.println("Connected to MQTT");

  timeClient.begin();
  Serial.println("Initiated time client");

  Serial.println("Initiating display");

  logInDisplay("Starting OS");
  randomSeed(micros());
  ArduinoOTA.begin();

  logInDisplay("Connecting to Blynk");
  dht.setup(DHT_PIN, DHTesp::DHT22);  // Connect DHT sensor to GPIO 17
  pinMode(DHT_PIN, INPUT_PULLUP);

  loadControlValues();
  logInDisplay("End of Setup");
  // initialize mDNS service
  esp_err_t err = mdns_init();
  if (err) {
    printf("MDNS Init failed: %d\n", err);
    return;
  }

  // set hostname
  mdns_hostname_set(HOSTNAME);
  // set default instance
  mdns_instance_name_set("Thermostat ESP32 C3");
  Serial.println("Starting OS v2");
}

int heatingSwitchH, thermostatTemperatureH, startHourWeekdayH, endHourWeekdayH,
    startHourWeekendH, endHourWeekendH;

// This is the main function which builds our GUI
void setUpUI() {
  // Turn off verbose debugging
  ESPUI.setVerbosity(Verbosity::Verbose);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = false;
  auto maintab = ESPUI.addControl(Tab, "", "Basic controls");

  heatingSwitchH = ESPUI.addControl(Switcher, "Heating main switch", "",
                                    Alizarin, maintab, generalCallback);

  thermostatTemperatureH = ESPUI.addControl(Slider, "Temperature", "", Alizarin,
                                            maintab, generalCallback);
  ESPUI.addControl(Min, "", "16", None, thermostatTemperatureH);
  ESPUI.addControl(Max, "", "24", None, thermostatTemperatureH);

  ESPUI.addControl(Separator, "General controls", "", None, maintab);

  startHourWeekdayH = ESPUI.addControl(Slider, "Start time weekdays", "",
                                       Turquoise, maintab, generalCallback);
  ESPUI.addControl(Min, "", "0", None, startHourWeekdayH);
  ESPUI.addControl(Max, "", "23", None, startHourWeekdayH);

  endHourWeekdayH = ESPUI.addControl(Slider, "End time weekdays", "", Turquoise,
                                     maintab, generalCallback);
  ESPUI.addControl(Min, "", "0", None, endHourWeekdayH);
  ESPUI.addControl(Max, "", "23", None, endHourWeekdayH);

  startHourWeekendH = ESPUI.addControl(Slider, "Start time weekend", "", Carrot,
                                       maintab, generalCallback);
  ESPUI.addControl(Min, "", "0", None, startHourWeekendH);
  ESPUI.addControl(Max, "", "23", None, startHourWeekendH);

  endHourWeekendH = ESPUI.addControl(Slider, "End time weekend", "", Carrot,
                                     maintab, generalCallback);
  ESPUI.addControl(Min, "", "0", None, endHourWeekendH);
  ESPUI.addControl(Max, "", "23", None, endHourWeekendH);

  /*
   * Tab: WiFi Credentials
   * You use this tab to enter the SSID and password of a wifi network to
   *autoconnect to.
   *-----------------------------------------------------------------------------------------------------------*/
  auto wifitab = ESPUI.addControl(Tab, "", "WiFi Credentials");
  wifi_ssid_text =
      ESPUI.addControl(Text, "SSID", "", Alizarin, wifitab, textCallback);
  // Note that adding a "Max" control to a text control sets the max length
  ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
  wifi_pass_text =
      ESPUI.addControl(Text, "Password", "", Alizarin, wifitab, textCallback);
  ESPUI.addControl(Max, "", "64", None, wifi_pass_text);
  ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab,
                   enterWifiDetailsCallback);

  // Finally, start up the UI.
  // This should only be called once we are connected to WiFi.
  ESPUI.begin(HOSTNAME);
}

String hour2str(int hour) {
  char numstr[2];  // enough to hold all numbers up to 64-bits
  sprintf(numstr, "%d", hour);
  return numstr;
}

void sendSensor() {
  float h = dht.getHumidity();
  float t = dht.getTemperature();
  if (isnan(h) || isnan(t)) {
    logMessage("Failed to read from DHT sensor!");
    return;
  }

  currentTemperature = t;
  if (mqttClient.connected()) {
    char numstr[45];
    sprintf(numstr, "{\"humidity\": %.2f, \"temperature\": %.2f}", h, t);
    mqttClient.publish(MQTT_SENSORS_TOPIC, numstr);
  }
}

void loop() {
  if (!mqttClient.connected()) {
    connectMqtt();
  }

  ArduinoOTA.handle();
  mqttClient.loop();
  timeClient.update();

  if (sensorsTimer.repeat()) {
    sendSensor();
    runtimeScenarios();
  }

#if !defined(ESP32)
  // We don't need to call this explicitly on ESP32 but we do on 8266
  MDNS.update();
#endif
}

void runtimeScenarios() {
  printDebugInformation();

  unsigned long currentMillis = millis();
  if (runlevelScenario == 0) {  // no scenario running
    logMessage("Init Runtime 0");
    if (areConditionsForRunlevel1Passed()) {  // wait for a time delay to make
                                              // sure there is correctly
                                              // measured avarage temperature
      runlevelScenario =
          gotoRunlevel1(currentTemperature, thermostatTemperature);
    }
  } else if (runlevelScenario == 1) {  // start scenario run level 1
    logMessage("Init Runtime 1");
    startScen = millis();
    scenLength = boilerOnTime(
        currentTemperature,
        thermostatTemperature);  // check how many seconds boiler should be on
                                 // in 10 minute interval
    runlevelScenario = 2;
    boilerStat = 1;                    // turn boiler on
  } else if (runlevelScenario == 2) {  // runlevel 2 always runs for 10 minutes
    logMessage("Init Runtime 2");
    if (boilerStat == 1) {  // onley check boiler stat if boiler is on this to
                            // prevent on/off fluctuation af ter overshoot
      boilerStat = checkOnBoiler(
          startScen, scenLength, millis(), currentTemperature,
          thermostatTemperature);  // continuesly check if boiler should be on
    }

    if (boilerStat == 1) {  // turn relay to on of boiler should be on
      logMessage("Boiler On");
      digitalWrite(RELAY_PIN, HIGH);
      // Blynk.virtualWrite(V6, HIGH);
    } else {
      logMessage("Boiler Off");
      digitalWrite(RELAY_PIN, LOW);
      // Blynk.virtualWrite(V6, LOW);
    }

    // after 10 minutes go back to run level 0
    if (currentMillis - startScen > maxScen) {
      runlevelScenario = 0;
    }
  }
}

boolean areConditionsForRunlevel1Passed() {
  return (heatingSwitch == 1) && (startHourWeekday != 0) &&
         (endHourWeekday != 0) && (thermostatTemperature != 0) &&
         (currentTemperature != 0) && isScheduleOn(isWeekend());
}

boolean isWeekend() {
  // 0 Sunday
  // 1-5 Monday to Friday
  // 6 Saturday
  return (timeClient.getDay() == 0 || timeClient.getDay() == 6) ? true : false;
}

boolean isScheduleOn(boolean isWeekend) {
  if (isWeekend) {
    return isScheduleOn(startHourWeekend, endHourWeekend);
  } else {
    return isScheduleOn(startHourWeekday, endHourWeekday);
  }
}

boolean isScheduleOn(int startHour, int endHour) {
  if (startHour >= endHour) {
    if (timeClient.getHours() > startHour) {
      return (timeClient.getHours() >= startHour);
    } else {
      return (timeClient.getHours() < endHour);
    }
  } else {
    return (timeClient.getHours() >= startHour) &&
           (timeClient.getHours() < endHour);
  }
}

// function to determine how many seconds boiler should go on within 10 minute
// interval
unsigned long boilerOnTime(int actTemp, int setTemp) {
  unsigned long scnel;
  // int iscnel = 0;
  logMessage("Temperature length sum: ", setTemp - actTemp);
  if (setTemp - actTemp > 5) {
    scnel = (9UL * 60UL * 1000UL);
  } else if (setTemp - actTemp > 3) {
    scnel = (7UL * 60UL * 1000UL);
  } else if (setTemp - actTemp > 2) {
    scnel = (5UL * 60UL * 1000UL);
  } else if (setTemp - actTemp > 1) {
    scnel = (3UL * 60UL * 1000UL);
  } else {
    scnel = (2UL * 60UL * 1000UL);
  }

  logMessage("Boiler goes on for: ", String(scnel));
  return scnel;
}

// function to determine if boiler controll loop should go to run level 1
int gotoRunlevel1(float currentTemperature, int thermostatTemperature) {
  logMessage("gotoRunlevel1");
  return thermostatTemperature > currentTemperature ? 1 : 0;
}

// function to check if boiler should stay on or go off
int checkOnBoiler(unsigned long starts, unsigned long expectedTimeOn,
                  unsigned long currentMillisOn, int currentTemperature,
                  int themorstatExpectedTemperature) {
  logMessage("Time boiler on: ", String(currentMillisOn - starts));
  logMessage("Expected time boiler on: ", String(expectedTimeOn));
  if (currentTemperature - themorstatExpectedTemperature <
      2) {  // criteria 1: only stay on if act temperature is below set
            // temperature + margin
    if (currentMillisOn - starts <
        expectedTimeOn) {  // criteria 2: only stay on of boiler has not been on
                           // for the number of seconds determined by
                           // fscenLength
      return 1;            // stay on
    } else {
      logMessage("Boiler has to go off");
      return 0;  // go off (because boiler was on for number of minues
                 // determined by fscenLength
    }
  } else {
    return 2;  // go off (2 is used to monitor overflow)
  }
}

void printDebugInformation() {
  logMessage("Current temnperature: ", currentTemperature);
  logMessage("Thermostat temnperature:", thermostatTemperature);
  logMessage("Start Hour during the week:", startHourWeekday);
  logMessage("End Hour during the week:", endHourWeekday);
  logMessage("Start Hour during the weekend:", startHourWeekend);
  logMessage("End Hour during the weekend:", endHourWeekend);
  logMessage("Heating system on:", heatingSwitch);
  logMessage("Is Scheduler On:", isScheduleOn(isWeekend()));
  logMessage("Is Weekend On:", isWeekend());
  logMessage("RAM:", String(ESP.getFreeHeap()));
}

void logMessage(String message, float toshow) {
  logMessage(message, String(toshow, 2));
}

void logMessage(String message, int toshow) {
  logMessage(message, String(toshow));
}

void logMessage(String message, boolean toshow) {
  logMessage(message, String(toshow));
}

void logInDisplay(String message) {
  //   display.clear();
  //   // Print to the screen
  //   display.println(message);
  //   // Draw it to the internal screen buffer
  //   display.drawLogBuffer(0, 0);
  //   // Display it on the screen
  //   display.display();

  logMessage(message);
}

void logMessage(String message) { logMessage(message, new String("")); }

void logMessage(String message, String toshow) {
  String finalMessage =
      timeClient.getFormattedTime() + ": " + message + " " + toshow;
  char arrayString[message.length() + 1];
  strcpy(arrayString, finalMessage.c_str());
  Serial.println(arrayString);
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic, arrayString);
  }
}

void connectWifi() {
  int connect_timeout;

#if defined(ESP32)
  WiFi.setHostname(HOSTNAME);
#else
  WiFi.hostname(HOSTNAME);
#endif
  Serial.println("Begin wifi...");

  // Load credentials from EEPROM
  if (!(FORCE_USE_HOTSPOT)) {
    yield();
    String stored_ssid, stored_pass;

    preferences.begin("my-app", false);
    stored_ssid = preferences.getString("ssid");
    stored_pass = preferences.getString("ssid_pass");
    preferences.end();

// Try to connect with stored credentials, fire up an access point if they don't
// work.
#if defined(ESP32)
    WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
#else
    WiFi.begin(stored_ssid, stored_pass);
#endif
    connect_timeout = 28;  // 7 seconds
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
      delay(250);
      Serial.print(".");
      connect_timeout--;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME)) {
      Serial.println("Error setting up MDNS responder!");
    }
  } else {
    Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);

    connect_timeout = 20;
    do {
      delay(250);
      Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

void connectMqtt() {
  if (!mqttClient.connected()) {
    logInDisplay("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = HOSTNAME + String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), NULL, NULL, 0, 2, 0, 0, 1)) {
      logMessage("Connected to MQTT");
      // Once connected, publish an announcement...
    } else {
      logMessage("failed, rc=", mqttClient.state());
    }
  }
}

void enterWifiDetailsCallback(Control *sender, int type) {
  if (type == B_UP) {
    Serial.println("Saving credentials to EPROM...");
    unsigned int i;
    preferences.begin("my-app", false);
    heatingSwitch =
        preferences.putString("ssid", ESPUI.getControl(wifi_ssid_text)->value);
    thermostatTemperature = preferences.putString(
        "ssid_pass", ESPUI.getControl(wifi_pass_text)->value);
    preferences.end();
  }
}

void saveControlValues() {
  preferences.begin("my-app", false);
  preferences.putUShort("hS", heatingSwitch);
  preferences.putUShort("tT", thermostatTemperature);
  preferences.putUShort("sHWday", startHourWeekday);
  preferences.putUShort("eHWday", endHourWeekday);
  preferences.putUShort("sHWend", startHourWeekend);
  preferences.putUShort("eHWend", endHourWeekend);
  preferences.end();

  Serial.print("Saved values in EEPROM, heatingSwitch: ");
  Serial.print(heatingSwitch);
  Serial.print(", thermostatTemperature: ");
  Serial.print(thermostatTemperature);
  Serial.print(", startHourWeekday: ");
  Serial.print(startHourWeekday);
  Serial.print(", endHourWeekday: ");
  Serial.print(endHourWeekday);
  Serial.print(", startHourWeekend: ");
  Serial.print(startHourWeekend);
  Serial.print(", endHourWeekend: ");
  Serial.println(endHourWeekend);

  //   loadControlValues();
}

void loadControlValues() {
  preferences.begin("my-app", false);
  heatingSwitch = preferences.getUShort("hS");
  thermostatTemperature = preferences.getUShort("tT");
  startHourWeekday = preferences.getUShort("sHWday");
  endHourWeekday = preferences.getUShort("eHWday");
  startHourWeekend = preferences.getUShort("sHWend");
  endHourWeekend = preferences.getUShort("eHWend");
  preferences.end();

  ESPUI.updateSwitcher(heatingSwitchH, heatingSwitch);
  ESPUI.updateSlider(thermostatTemperatureH, thermostatTemperature);
  ESPUI.updateSlider(startHourWeekdayH, startHourWeekday);
  ESPUI.updateSlider(endHourWeekdayH, endHourWeekday);
  ESPUI.updateSlider(startHourWeekendH, startHourWeekend);
  ESPUI.updateSlider(endHourWeekendH, endHourWeekend);
}

void textCallback(Control *sender, int type) { saveControlValues(); }

void generalCallback(Control *sender, int type) {
  if (sender->id == heatingSwitchH)
    heatingSwitch = sender->value.toInt();
  else if (sender->id == thermostatTemperatureH)
    thermostatTemperature = sender->value.toInt();
  else if (sender->id == startHourWeekdayH)
    startHourWeekday = sender->value.toInt();
  else if (sender->id == endHourWeekdayH)
    endHourWeekday = sender->value.toInt();
  else if (sender->id == startHourWeekendH)
    startHourWeekend = sender->value.toInt();
  else if (sender->id == endHourWeekendH)
    endHourWeekend = sender->value.toInt();
  else {
    Serial.print("Unexpected value for label");
    Serial.println(sender->label);
  }

  Serial.print("CB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);

  saveControlValues();
}

void randomString(char *buf, int len) {
  for (auto i = 0; i < len - 1; i++) buf[i] = random(0, 26) + 'A';
  buf[len - 1] = '\0';
}
