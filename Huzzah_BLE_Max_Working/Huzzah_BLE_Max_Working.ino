/*---------------------------------------------------------------------------------------------

  Open Sound Control (OSC) library for the ESP8266

  Example for receiving open sound control (OSC) bundles on the ESP8266
  Send integers '0' or '1' to the address "/led" to turn on/off the built-in LED of the esp8266.

  This example code is in the public domain.

--------------------------------------------------------------------------------------------- */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
#include <SparkFunBLEMate2.h>
#include <SoftwareSerial.h>

//#define DEBUG
//#ifdef DEBUG
//#endif

#define RIBCAGE 16
#define SPINE 15
#define MIDBACK 15
#define LOWBACK 15
#define WRISTL 15
#define WRISTR 15
#define ANKLEL 15
#define ANKLER 15

SoftwareSerial DebugSerial(12,13);
BLEMate2 BTModu(&Serial);

char ssid[] = "BROOKLYN RESEARCH";          // your network SSID (name)
char pass[] = "pizza247";  // your network password

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
const unsigned int localPort = 7400;  // local port to listen for UDP packets (here's where we send the packets)
const int SmartBasicPin = 2;
int scaleInt = 0;               // scale for incoming values. 0-255 input * 16 = 0-4096 output
int address;
int values[7];

void printDebug(String message, OSCMessage &msg) {
//   return;
    Serial.println("--------------------");
    Serial.println("");
    Serial.print(message); Serial.print(msg.getInt(0));Serial.println("");
    Serial.println("--------------------");
    Serial.println("");
}



//*** Functions for SmartBasic
void selectPC(){
  Serial.flush();
  digitalWrite(SmartBasicPin, LOW);
}
void selectBLE(){
  Serial.flush();
  digitalWrite(SmartBasicPin,HIGH);
}
void setup() {
  //Serial.begin(460800);
  pinMode(SmartBasicPin, OUTPUT);
  pinMode(RIBCAGE, OUTPUT);  //MORE PINS!!!
  analogWrite(RIBCAGE, 0);
  Serial.begin(9600);
  while (!Serial) { ; // wait for serial port to connect. Needed for native USB port only
  }      

  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");  
    }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  
  //*** Serial for BLE
  DebugSerial.begin(115200);
  DebugSerial.println("Please wait...");
  //*** Setting up BLE
  selectBLE();           // Route serial data to the BC118.
  if (BTModu.reset() != BLEMate2::SUCCESS)
  {
    selectPC();
    DebugSerial.println("Module reset error!");
    while (1);  }
  if (BTModu.restore() != BLEMate2::SUCCESS)
  {
    selectPC();
    DebugSerial.println("Module restore error!");
    while (1);  }
  if (BTModu.writeConfig() != BLEMate2::SUCCESS)
  {
    selectPC();
    DebugSerial.println("Module write config error!");
    while (1);  }
  // One more reset, to make the changes take effect.
  if (BTModu.reset() != BLEMate2::SUCCESS)
  {
    selectPC();
    DebugSerial.println("Second module reset error!");
    while (1);
  }
  selectBLE();
  setupPeripheralExample();
}

//*** Peripheral BLE
void setupPeripheralExample()
{
  boolean inCentralMode = false;
  BTModu.amCentral(inCentralMode); 
  if (inCentralMode)
  {
    BTModu.BLEPeripheral();
    BTModu.BLEAdvertise();
  }
  // The CCON parameter will enable advertising immediately after a disconnect.
  BTModu.stdSetParam("CCON", "ON");
  // The ADVP parameter controls the advertising rate. Can be FAST or SLOW.
  BTModu.stdSetParam("ADVP", "SLOW");
  // The ADVT parameter controls the timeout before advertising stops. Can be
  //  0 (for never) to 4260 (71min); integer value, in seconds.
  BTModu.stdSetParam("ADVT", "0");
  // The ADDR parameter controls the devices we'll allow to connect to us.
  //  All zeroes is "anyone".
  BTModu.stdSetParam("ADDR", "000000000000");

  BTModu.writeConfig();
  BTModu.reset();

  DebugSerial.println(F("\nYou can connect your device now"));
}

//*** Setting Pins
void ribcage(OSCMessage &msg) {
  printDebug("RIB CAGE: ", msg);
  motor(RIBCAGE, msg.getInt(0));
}

void spine(OSCMessage &msg) {
  printDebug("SPINE: ", msg);
  motor(SPINE, msg.getInt(0));
}

void midback(OSCMessage &msg) {
  printDebug("MIDBACK: ", msg);
  motor(MIDBACK, msg.getInt(0));
}

void lowback(OSCMessage &msg) {
  printDebug("LOWBACK: ", msg);
  motor(LOWBACK, msg.getInt(0));
}

void wristL(OSCMessage &msg) {
  printDebug("LEFT WRIST: ", msg);
  motor(WRISTL, msg.getInt(0));
}

void wristR(OSCMessage &msg) {
  printDebug("RIGHT WRIST: ", msg);
  motor(WRISTR, msg.getInt(0));
}

void ankleL(OSCMessage &msg) {
  printDebug("LEFT ANKLE: ", msg);
  motor(ANKLEL, msg.getInt(0));
}

void ankleR(OSCMessage &msg) {
  printDebug("RIGHT ANKLE: ", msg);
  motor(ANKLER, msg.getInt(0));
}

void test(OSCMessage &msg) {
  printDebug("TEST: ", msg);
}

void motor(int pin, int scale){   
  if(scale >= 0){   
      analogWrite(pin, scale);
  }
}

void loop() {
  //*** OSC messages
  OSCMessage msg;
  OSCErrorCode error;

  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/ribcage", ribcage);
      msg.dispatch("/spine", spine);
      msg.dispatch("/midback", midback);
      msg.dispatch("/lowback", lowback);
      msg.dispatch("/wristL", wristL);
      msg.dispatch("/wristR", wristR);
      msg.dispatch("/ankleL", ankleL);
      msg.dispatch("/ankleR", ankleR);
      msg.dispatch("/test", test);
//      Serial.print("error: "); 
//      Serial.println(error);   
    }
  }

  //*** BLE messages
  static String fullBuffer = "";
  static long lastRXTime = millis();
  static String s = "";
  if (lastRXTime + 1000 < millis())
  {
    if (fullBuffer != "")
    {
      selectPC();  
      fullBuffer.trim();
      fullBuffer.toLowerCase();    
      if(fullBuffer.startsWith("id"))
      {
        fullBuffer.trim();
        fullBuffer.remove(0,2);
        fullBuffer.trim();
        address = fullBuffer.toInt();
        DebugSerial.print(F("New Address is: #")); 
        DebugSerial.println(fullBuffer); 
        }
        else {
          scaleInt = fullBuffer.toInt();
          if(0 <= scaleInt && scaleInt <= 255){
            DebugSerial.print(F("Scale = ")); 
            DebugSerial.println(scaleInt * 16); 
              //motor(scaleInt);
          }
          else{
            DebugSerial.print(F("! Please Enter scale in the range 0-255 !")); 
            }
        }
      selectBLE();
      fullBuffer = "";
    }
  }
  static String inputBuffer;
  while (Serial.available() > 0)
  {   
      inputBuffer.concat((char)Serial.read());
      lastRXTime = millis();
      if (inputBuffer.endsWith("\n\r"))
      {
        if (inputBuffer.startsWith("RCV="))
        {
          inputBuffer.trim(); // Remove \n\r from end.
          inputBuffer.remove(0,4); // Remove RCV= from front.
          fullBuffer += inputBuffer;
          inputBuffer = "";
        }
        else
        {
          inputBuffer = "";
        }
      }
  }

  // SEND
  if (DebugSerial.available()) 
  {
      DebugSerial.setTimeout(100); 
      s = DebugSerial.readString();
      char ch[20];
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      s.toCharArray(ch, 20);
      
      DebugSerial.print(F("\n* Sending -> ")); 
      DebugSerial.print((char *) sendbuffer); 
     // motor(s.toInt());
      BTModu.sendData(s);
   }
   
    
}
