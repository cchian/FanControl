#include <Preferences.h>
Preferences preferences;
uint8_t nodeId;
#define BAUDRATE 115200

#define SPINDLE_SPEED_1 12
#define SPINDLE_SPEED_2 14
#define SPINDLE_SPEED_3 26

#define SPINDLE_SWING_PIN 27

#define IR_RECV_PIN  5

//Switch working function
#define BUTTON_PIN 23
//#define BUTTON_PIN 16 //test
#define INDICATOR_PIN 15


#define TCP_PORT              1988

#define FILESYSTEM SPIFFS
#if FILESYSTEM == FFat
#include <FFat.h>
#endif

#if FILESYSTEM == SPIFFS
#include <SPIFFS.h>
#endif

#include <Button.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <Update.h>
#include <WebSocketsServer.h>
WebSocketsServer webSocket = WebSocketsServer(TCP_PORT);
bool stringComplete = false;
String inputString = "";
uint8_t socketNum;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  String sMsg = "";
  switch (type) {
    case WStype_DISCONNECTED:
      // Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        //IPAddress ip = webSocket.remoteIP(num);
        //Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        //webSocket.sendTXT(num, "Connected");
      }
      break;
    case WStype_TEXT:
      for (int i = 0; i < length; i++)
        sMsg += (char)payload[i];
#if defined(ARDUINO_ESP8266_NODEMCU) || defined(ESP32)
      inputString = sMsg;
      stringComplete = true;
      socketNum = num;

      Serial.println("Socket.receive(" + sMsg + ")");
#else
      Serial.println("Socket.receive(" + sMsg + ")");
#endif
      // send message to client
      // webSocket.sendTXT(num, "message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("message here");
      break;
    case WStype_BIN:;
      //Serial.printf("[%u] get binary length: %u\n", num, length);
      //hexdump(payload, length);

      // send message to client
      // webSocket.sendBIN(num, payload, length);
      break;
  }
}
#include <PubSubClient.h>

#define DEVICE_NAME "FAN_1"
const char* host = "FAN_";
#define HOME_MANAGER "homage"
#define VERSIONCODE "1"

char mqtt_server[] = "broker.hivemq.com";
int mqtt_port = 1883;
char mqtt_username[] = "";
char mqtt_password[] = "";

#define ssid "lloUuou2"
#define pass "34567890"
int wf_connect_timeout = 15;

Button btn(BUTTON_PIN, DIO, HIGH);

TaskHandle_t ButtonTask;    // Task 6

#define Version "1"  // sketch version //

long intervals = 0;
volatile long intervals_remaining = 0;
volatile boolean isRunning = false;

// comm variables
const int MAX_CMD_SIZE = 64;
char buffer[MAX_CMD_SIZE]; // buffer for serial commands
char serial_char; // value for each byte read in from serial comms
int serial_count = 0; // current length of command
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
boolean comment_mode = false;

int next_command_request_counter = 0;   //if this counter reaches the maximum then a "ok" is sent to request the nex command
int next_command_request_maximum = 1000;
// end comm variables

// GCode States
double currentOffsetX = 0.;
double currentOffsetY = 0.;
double currentOffsetA = 0.;
double currentOffsetB = 0.;

int currentSpeed;
int currentSwing = false;

boolean absoluteMode = true;
double feedrate = 10; // mm/minute

WebServer webServer(80);
WiFiClient wifiClient;
PubSubClient client(wifiClient);


#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  5        //Time ESP32 will go to sleep (in seconds)

RTC_DATA_ATTR int bootCount = 0;


bool testDigit(int input, int test) {
  return (input == test) ? 1 : 0;
}
void updateState(int spd = currentSpeed) {
  spd = (spd > 3) ? 0 : spd;
  spd = (spd < 0) ? 3 : spd;
  currentSpeed = spd;

  digitalWrite(SPINDLE_SPEED_1, testDigit(currentSpeed, 1));
  digitalWrite(SPINDLE_SPEED_2, testDigit(currentSpeed, 2));
  digitalWrite(SPINDLE_SPEED_3, testDigit(currentSpeed, 3));
  (currentSpeed > 0) ? digitalWrite(SPINDLE_SWING_PIN, currentSwing) : digitalWrite(SPINDLE_SWING_PIN, 0);

  switch (currentSpeed) {
    case 0:
      digitalWrite(SPINDLE_SPEED_1, 0);
      digitalWrite(SPINDLE_SPEED_2, 0);
      digitalWrite(SPINDLE_SPEED_3, 0);
      digitalWrite(SPINDLE_SWING_PIN, 0);
      break;
    case 1:
      digitalWrite(SPINDLE_SPEED_1, 1);
      digitalWrite(SPINDLE_SPEED_2, 0);
      digitalWrite(SPINDLE_SPEED_3, 0);
      digitalWrite(SPINDLE_SWING_PIN, currentSwing);
      break;
    case 2:
      digitalWrite(SPINDLE_SPEED_1, 0);
      digitalWrite(SPINDLE_SPEED_2, 1);
      digitalWrite(SPINDLE_SPEED_3, 0);
      digitalWrite(SPINDLE_SWING_PIN, currentSwing);
      break;
    case 3:
      digitalWrite(SPINDLE_SPEED_1, 0);
      digitalWrite(SPINDLE_SPEED_2, 0);
      digitalWrite(SPINDLE_SPEED_3, 1);
      digitalWrite(SPINDLE_SWING_PIN, currentSwing);
      break;
  }
  Serial.println("Speed:" + String(currentSpeed));
  Serial.println("Swing:" + String((currentSwing == 1) ? "ON" : "OFF"));
}


void press(int sender) {
  //Serial.println("Pressed! pin " + String(sender));
  
}
void click(int sender) {
  //Serial.println("Clicked! pin " + String(sender));
  updateState(currentSpeed + 1);
}
void doubleClick(int sender) {
  //Serial.println("Double Clicked! pin " + String(sender));
  currentSwing = !currentSwing;
  updateState();
}
void hold(int sender) {
  //Serial.println("Holded! pin " + String(sender));
  updateState(0);
}
void longHold(int sender) {
  Serial.println("Long Holded! pin " + String(sender));
}
void initButton() {
  //set hold time to 1 second.
  btn.setDoubleClickTime(200);
  btn.setHoldTime(1000);

  //btn.eventPress((void*)press);
  btn.eventClick((void*)click);
  btn.eventDoubleClick((void*)doubleClick);
  btn.eventHold((void*)hold);
  btn.eventLongHold((void*)longHold);

  xTaskCreate(&Button_Task, "ButtonTASK", 2048, NULL, 14, &ButtonTask);
}

void Button_Task(void *p) {
  while (1) {
    btn.handleButton();
    delay(1);
  }
}


void setup() {
  Serial.begin(BAUDRATE);
  Serial.print(VERSIONCODE);
  Serial.print("\n");
 
  preferences.begin("p", false);   
  nodeId = preferences.getUChar("id");
  //preferences.putUChar("id", 2);
  preferences.end();
  
  pinMode(SPINDLE_SPEED_1, OUTPUT);
  pinMode(SPINDLE_SPEED_2, OUTPUT);
  pinMode(SPINDLE_SPEED_3, OUTPUT);

  pinMode(SPINDLE_SWING_PIN, OUTPUT);

  pinMode(INDICATOR_PIN, OUTPUT);
  clear_buffer();

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == 4) {
    Serial.println("Wakeup caused by touchpad");
    //use AP for offline
    WiFi.softAP((String(host)+String(nodeId)).c_str(), "");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
  } else {
    Serial.println("Wakeup was not caused by deep sleep");
    //use STA for online
    WiFi.mode(WIFI_STA);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    btStop();
    if (WiFi.begin(ssid, pass)) {
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        wf_connect_timeout--;
        if (wf_connect_timeout == 0)
          esp_deep_sleep_start();
        Serial.print(".");
      }
    }
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  if (client.connect(DEVICE_NAME, mqtt_username, mqtt_password)) {
    Serial.println("connected");
    client.subscribe(DEVICE_NAME);
  }
  
  initButton();
  initHandleds();
  webServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.print(F("TCP PORT:"));
  Serial.println(String(TCP_PORT));
}

void callback(char* topic, byte* message, unsigned int length) {

  String messageTemp;
  for (int i = 0; i < length; i++) {
    // Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  messageTemp.toUpperCase();
  //   Serial.print(messageTemp); Serial.print(">>"); Serial.println(String(messageTemp).length());

  messageTemp.toCharArray(buffer, String(messageTemp).length() + 1);
  serial_count = String(messageTemp).length() + 1;
  next_command_request_counter = 0;
  buffer[serial_count] = 0;
  process_commands(buffer, serial_count + 1);
  clear_buffer();
  comment_mode = false;

  // Serial.println("mqtt callback>>>>>>"+messageTemp);
}

void loop() { // input loop, looks for manual input and then checks to see if and serial commands are coming in
  get_command(); // check for Gcodes
  webServer.handleClient();
  client.loop();
}

void clear_buffer() { // empties command buffer from serial connection
  serial_count = 0; // reset buffer placement
}

void get_command() { // gets commands from serial connection and then calls up subsequent functions to deal with them
  if (!isRunning && Serial.available() > 0) { // each time we see something
    serial_char = Serial.read(); // read individual byte from serial connection
    if (serial_char >= 97) {
      serial_char -= 32;
    }
    if (serial_char == '\n' || serial_char == '\r') { // end of a command character
      next_command_request_counter = 0;
      buffer[serial_count] = 0;
      process_commands(buffer, serial_count);
      clear_buffer();
      comment_mode = false; // reset comment mode before each new command is processed
      //Serial.write("process: command");
    } else { // not end of command
      if (serial_char == ';' || serial_char == '(') // semicolon signifies start of comment
        comment_mode = true;
      if (comment_mode != true) { // ignore if a comment has started
        buffer[serial_count] = serial_char; // add byte to buffer string
        serial_count++;
        if (serial_count > MAX_CMD_SIZE) { // overflow, dump and restart
          clear_buffer();
          Serial.flush();
          Serial.print("Overflow Error\n");
        }
      }
    }
  }
}

boolean getValue(char key, char command[], double* value) {
  // find key parameter
  strchr_pointer = strchr(buffer, key);
  if (strchr_pointer != NULL) { // We found a key value
    *value = (double)strtod(&command[strchr_pointer - command + 1], NULL);
    return true;
  }
  return false;
}

void check_for_version_controll(char command) {
  if (command == 'V') {
    Serial.print(VERSIONCODE);
    Serial.print("\n");
    //    client.publish(DEVICE_NAME, String(VERSIONCODE));
  }
}




void process_commands(char command[], int command_length) { // deals with standardized input from serial connection
  double nVal;
  boolean hasNVal = getValue('N', command, &nVal);
  //if (hasNVal) {Serial.println("linenumber detected");};

  double getcs;
  boolean hasCS = getValue('*', command, &getcs);
  //if (hasCS) {Serial.println("checksum detected");};

  // checksum code from reprap wiki
  int cs = 0;
  int j = 0;


  for (j = 0; command[j] != '*' && command[j] != 0; j++)
    cs = cs ^ command[j];
  cs &= 0xff;  // Defensive programming...

  if (!(cs == (int)getcs || hasCS == false)) { // if checksum does not match
    Serial.print("rs ");
    Serial.print((int)getcs);
    //Serial.print((int)nVal);
    Serial.print("\n");
    //Serial.flush();
  } else if (getValue('?', command, &nVal)) {
    String datas = "From:" + String(DEVICE_NAME) + ",Speed:" + String(currentSpeed) + ",Swing:" + String((currentSwing == 1) ? "ON" : "OFF");
    Serial.println(datas);
    byte* p = (byte*)malloc(datas.length());
    memcpy(p, datas.c_str(), datas.length());
    client.publish( (String(DEVICE_NAME) + "_Reply").c_str() , p, datas.length());
  } else {
    //continue if checksum matches or none detected
    //Serial.println("checksum match ");
    j = 0;
    while (j < MAX_CMD_SIZE ) {
      if ((command[j] == 'G') || command[j] == 'M') {
        break;
      }
      j++;
    }

    if (command_length == 1) {
      check_for_version_controll(command[0]);
    }
    if (command_length > 0 && command[j] == 'G') { // G code
      //Serial.print("process G: \n");
      int codenum = (int)strtod(&command[j + 1], NULL);


      double xVal;
      boolean hasXVal = getValue('X', command, &xVal);
      // if (hasXVal) xVal *= zoom*xscaling;

      double yVal;
      boolean hasYVal = getValue('Y', command, &yVal);
      // if (hasYVal) yVal *= zoom*yscaling;

      double aVal;
      boolean hasAVal = getValue('A', command, &aVal);
      // if (hasAVal) aVal *= zoom*ascaling;

      double bVal;
      boolean hasBVal = getValue('B', command, &bVal);
      // if (hasBVal) bVal *= zoom*bscaling;

      double zVal;
      boolean hasZVal = getValue('Z', command, &zVal);

      double iVal;
      boolean hasIVal = getValue('I', command, &iVal);
      //  if (hasIVal) iVal *= zoom;

      double jVal;
      boolean hasJVal = getValue('J', command, &jVal);
      // if (hasJVal) jVal *= zoom;

      double rVal;
      boolean hasRVal = getValue('R', command, &rVal);
      //  if (hasRVal) rVal *= zoom;

      double pVal;
      boolean hasPVal = getValue('P', command, &pVal);

      getValue('F', command, &feedrate);



      xVal += currentOffsetX;
      yVal += currentOffsetY;

      aVal += currentOffsetA;
      bVal += currentOffsetB;



      switch (codenum) {
        case 0: // G0, Rapid positioning

          if (hasXVal) {
            Serial.println("----->X" + String(xVal));

          }

          if (hasYVal) {

          }
          if (hasAVal) {

          }
          if (hasBVal) {

          }


          break;
        case 1: // G1, linear interpolation at specified speed

          if (hasXVal) {

          }

          if (hasYVal) {

          }
          if (hasAVal) {

          }
          if (hasBVal) {

          }


          break;
        case 2: // G2, Clockwise arc
        case 3: // G3, Counterclockwise arc
          if (hasIVal && hasJVal) {
            //   double centerX=xAxisStepper.getCurrentPosition()+iVal;
            //    double centerY=rotationStepper.getCurrentPosition()+jVal;
            //    drawArc(centerX, centerY, tempX, tempY, (codenum==2));
          } else if (hasRVal) {
            //drawRadius(tempX, tempY, rVal, (codenum==2));
          }
          break;
        case 4: // G4, Delay P ms
          if (hasPVal) {
            unsigned long endDelay = millis() + (unsigned long)pVal;
            while (millis() < endDelay) {
              delay(1);


            }
          }
          break;
        case 21: // G21 metric
          break;
        case 90: // G90, Absolute Positioning
          absoluteMode = true;
          break;
        case 91: // G91, Incremental Positioning
          absoluteMode = false;
          break;
        case 92: // G92 homing
          break;
      }
    } else if (command_length > 0 && (command[j] == 'M' || command[j] == 'm')) { // M code
      //Serial.print("proces M:\n");
      double value;
      int codenum = (int)strtod(&command[j + 1], NULL);

      double sVal;
      boolean hasSpeedValue = getValue('S', command, &sVal);

      double pVal;
      boolean hasPValue = getValue('P', command, &pVal
                                  );
      switch (codenum) {
        case 3:
        case 4:
          if (hasSpeedValue) {
            currentSpeed = (int)sVal;
          }

          if (hasPValue) {
            if ((int)pVal == 1) {
              digitalWrite(SPINDLE_SWING_PIN, 1);
              currentSwing = 1;
            } else {
              digitalWrite(SPINDLE_SWING_PIN, 0);
              currentSwing = 0;
            }
          }
          updateState(currentSpeed);
          break;
        case 5:
          updateState(0);
          Serial.println("FAN OFF");
          break;
        case 8:
          if (getValue('S', command, &value) && (value >= 0 && value <= 255)) {

          } else {

          }
          break;
        case 9:

          break;
        case 88:
          if (getValue('S', command, &value) && (value >= 0 && value <= 255)) {
            //digitalWrite(DISC, HIGH);

          }
          break;
        case 99:

          break;
        case 18: // Disable Drives
          //          xAxisStepper.resetStepper();
          //          rotationStepper.resetStepper();
          //
          //          aAxisStepper.resetStepper();
          //          bAxisStepper.resetStepper();
          break;

        case 300: // Servo Position
          if (getValue('S', command, &value)) {

          }
          break;

        case 400: // Propretary: Reset X-Axis-Stepper settings to new object diameter
          if (getValue('S', command, &value)) {
            //            xAxisStepper.resetSteppersForObjectDiameter(value);
            //            xAxisStepper.setTargetPosition(0.);
            //            commitSteppers(maxFeedrate);
            //            delay(2000);
            // xAxisStepper.enableStepper(false);
          }
          break;

        case 401: // Propretary: Reset Y-Axis-Stepper settings to new object diameter
          if (getValue('S', command, &value)) {
            //            rotationStepper.resetSteppersForObjectDiameter(value);
            //            rotationStepper.setTargetPosition(0.);
            //            commitSteppers(maxFeedrate);
            //            delay(2000);
            //            rotationStepper.enableStepper(false);
          }
          break;

        case 402: // Propretary: Reset Y-Axis-Stepper settings to new object diameter
          if (getValue('S', command, &value)) {

          }
          break;

        default:
          break;
      }
    }

    //done processing commands
    //if (Serial.available() <= 0) {
    Serial.print("ok '");
    client.publish(HOME_MANAGER, "OK");
    //Serial.print((int)getcs);
    Serial.print(command);
    Serial.print("'\n");
    //Serial.flush();
    //}
  }
}
