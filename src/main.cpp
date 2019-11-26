// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
#include <main.h>
char auth[] = BLYNK_AUTH;
#define BLYNK_PRINT Serial

const char *ssid = SSID_NAME;
const char *password = SSID_PASSWORD;
const char *mqtt_server = MQTT_SERVER;
const char *mqtt_topic = MQTT_TOPIC;
const int DHTPin = 4;
const int relayPin = 12;
const unsigned long maxScen = 600000;  // 10 minutes

#define DHTTYPE DHT22

// WiFi includes
#include <ESP8266WiFi.h>
#include <NTPClient.h>
// #include <Blynk.h>
#include <BlynkSimpleEsp8266.h>
// OTA Includes
#include <ArduinoOTA.h>
#include <DHTesp.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <Wire.h>         // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"  // legacy include: `#include "SSD1306.h"`

void reconnectPlatform();
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
int gotoRunlevel1(int actualTemperature, int thermostatTemperature);
String hour2str(int hour);

int hittingOn = 0;
int startHourWeekday = 0;
int endHourWeekday = 0;

int startHourWeekend = 0;
int endHourWeekend = 0;

int thermostatTemperature = 0;
float currentTemperature = 0;
int runlevelScenario = 0;
unsigned long startScen = 0;   // scenario started
int boilerStat = 0;            // boiler status
unsigned long scenLength = 0;  // minutes on

WiFiUDP ntpUDP;
DHTesp dht;
BlynkTimer sensorsTimer;
BlynkTimer runtimeTimer;
BlynkTimer displayTimer;
BlynkTimer reconnectPlatformTimer;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 3600000);
WiFiClient espClient;
PubSubClient client(espClient);

SSD1306Wire display(0x3c, D3, D5);

void setup() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 10,
                     "Starting OS");
  display.display();

  display.setLogBuffer(5, 30);

  WiFi.begin(ssid, password);
  Serial.begin(9600);
  timeClient.begin();
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
  }

  logInDisplay("Connected to WIFI");
  client.setServer(mqtt_server, 1883);

  logInDisplay("Connected to MQTT");
  display.display();
  randomSeed(micros());
  ArduinoOTA.begin();
  ArduinoOTA.onStart([]() {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 10,
                       "OTA Update");
    display.display();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    display.drawProgressBar(4, 32, 120, 8, progress / (total / 100));
    display.display();
  });

  ArduinoOTA.onEnd([]() {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(display.getWidth() / 2, display.getHeight() / 2,
                       "Restarting the system");
    display.display();
  });
  logInDisplay("Connecting to Blynk");
  Blynk.begin(auth, ssid, password);

  // Hitting on off
  Blynk.syncVirtual(V0);

  // Start hours in which it work
  Blynk.syncVirtual(V3);

  // Stop hours in which it work
  Blynk.syncVirtual(V4);

  // Thermostat temperatura
  Blynk.syncVirtual(V5);

  // V6 is the boiler

  // Start hours weekends
  Blynk.syncVirtual(V7);

  // Stop hours weekends
  Blynk.syncVirtual(V8);

  dht.setup(4, DHTesp::DHT22);  // Connect DHT sensor to GPIO 17
  // Setup a function to be called every 10 seconds
  sensorsTimer.setInterval(10000L, sendSensor);
  runtimeTimer.setInterval(10000L, runtimeScenarios);
  displayTimer.setInterval(1000L, updateScreen);
  reconnectPlatformTimer.setInterval(30000L, reconnectPlatform);

  reconnectPlatform();
  logInDisplay("End of Setup");
}

BLYNK_WRITE(V0)  // Button Widget is writing to pin V1
{
  int pinData = param.asInt();
  if (pinData == HIGH) {
    hittingOn = 1;
  } else {
    hittingOn = 0;
  }
}

void reconnectPlatform() {
  int wifitries = 0;
  while ((WiFi.status() != WL_CONNECTED)) {
    logInDisplay("Reconnecting to Wifi!");
    WiFi.begin(ssid, password);
    wifitries++;
    delay(500);

    if (wifitries == 5) {
      logInDisplay("Failed after 5 attempts!");
      return;
    }
  }

  if (!Blynk.connected()) {
    logInDisplay("Not connected to Blynk server");
    Blynk.connect();  // try to connect to server with default timeout
  } else {
    Serial.println("Connected to Blynk server");
  }

  // Loop until we're reconnected
  if (!client.connected()) {
    logInDisplay("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), NULL, NULL, 0, 2, 0, 0, 1)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}

void updateScreen() {
  String topText =
      "M-F: " + hour2str(startHourWeekday) + "-" + hour2str(endHourWeekday);
  String bottomText =
      "SS: " + hour2str(startHourWeekend) + "-" + hour2str(endHourWeekend);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, topText);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 17, bottomText);

  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  char buffer[50];
  sprintf(buffer, "%.1f", currentTemperature);
  display.drawString(128, 40, buffer);

  display.display();
}

String hour2str(int hour) {
  char numstr[2];  // enough to hold all numbers up to 64-bits
  sprintf(numstr, "%d", hour);
  return numstr;
}

BLYNK_WRITE(V3) { startHourWeekday = param.asInt(); }

BLYNK_WRITE(V4) { endHourWeekday = param.asInt(); }

BLYNK_WRITE(V5) { thermostatTemperature = param.asInt(); }

BLYNK_WRITE(V7) { startHourWeekend = param.asInt(); }

BLYNK_WRITE(V8) { endHourWeekend = param.asInt(); }

void sendSensor() {
  float h = dht.getHumidity();
  float t =
      dht.getTemperature();  // or dht.readTemperature(true) for Fahrenheit
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    Blynk.virtualWrite(V1, -1);
    Blynk.virtualWrite(V2, -1);
    return;
  }

  currentTemperature = t;

  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V2, t);
}

void loop() {
  ArduinoOTA.handle();
  client.loop();
  Blynk.run();

  sensorsTimer.run();
  runtimeTimer.run();
  displayTimer.run();
  reconnectPlatformTimer.run();
  timeClient.update();
}

void runtimeScenarios() {
  printDebugInformation();

  unsigned long currentMillis = millis();
  if (runlevelScenario == 0) {  // no scenario running
    Serial.println("Init Runtime 0");
    if (areConditionsForRunlevel1Passed()) {  // wait for a time delay to make
                                              // sure there is correctly
                                              // measured avarage temperature
      runlevelScenario =
          gotoRunlevel1(currentTemperature, thermostatTemperature);
    }
  } else if (runlevelScenario == 1) {  // start scenario run level 1
    Serial.println("Init Runtime 1");
    startScen = millis();
    scenLength = boilerOnTime(
        currentTemperature,
        thermostatTemperature);  // check how many seconds boiler should be on
                                 // in 10 minute interval
    runlevelScenario = 2;
    boilerStat = 1;                    // turn boiler on
  } else if (runlevelScenario == 2) {  // runlevel 2 always runs for 10 minutes
    Serial.println("Init Runtime 2");
    if (boilerStat == 1) {  // onley check boiler stat if boiler is on this to
                            // prevent on/off fluctuation af ter overshoot
      boilerStat = checkOnBoiler(
          startScen, scenLength, millis(), currentTemperature,
          thermostatTemperature);  // continuesly check if boiler should be on
    }

    if (boilerStat == 1) {  // turn relay to on of boiler should be on
      Serial.println("Boiler On");
      digitalWrite(relayPin, HIGH);
      Blynk.virtualWrite(V6, HIGH);
    } else {
      Serial.println("Boiler Off");
      digitalWrite(relayPin, LOW);
      Blynk.virtualWrite(V6, LOW);
    }

    // after 10 minutes go back to run level 0
    if (currentMillis - startScen > maxScen) {
      runlevelScenario = 0;
    }
  }
}

boolean areConditionsForRunlevel1Passed() {
  return (hittingOn == 1) && (startHourWeekday != 0) && (endHourWeekday != 0) &&
         (thermostatTemperature != 0) && (currentTemperature != 0) &&
         isScheduleOn(isWeekend());
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
int gotoRunlevel1(int actualTemperature, int thermostatTemperature) {
  logMessage("gotoRunlevel1");
  if (thermostatTemperature - actualTemperature > 2) {
    return 1;
  } else {
    return 0;
  }
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
  logMessage("Hitting system on:", hittingOn);
  logMessage("Is Scheduler On:", isScheduleOn(isWeekend()));
  logMessage("Is Weekend On:", isWeekend());
  logMessage("RAM:", String(ESP.getFreeHeap()));
}

void logMessage(String message, float toshow) {
  logMessage(message, String(toshow, 2));
}

void logMessage(String message) { logMessage(message, ""); }

void logMessage(String message, int toshow) {
  logMessage(message, String(toshow));
}

void logMessage(String message, boolean toshow) {
  logMessage(message, String(toshow));
}

void logInDisplay(String message) {
  display.clear();
  // Print to the screen
  display.println(message);
  // Draw it to the internal screen buffer
  display.drawLogBuffer(0, 0);
  // Display it on the screen
  display.display();
}

void logMessage(String message, String toshow) {
  String finalMessage = timeClient.getFormattedTime() + ": " + message + toshow;
  char arrayString[message.length() + 1];
  strcpy(arrayString, finalMessage.c_str());
  Serial.println(arrayString);
  if (client.connected()) {
    client.publish(mqtt_topic, arrayString);
  }
}