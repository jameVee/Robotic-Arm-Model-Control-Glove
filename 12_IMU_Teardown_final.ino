#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <M5StickC.h>
//#include "M5StickCPlus.h"

#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include <HTTPClient.h>

#define LEDPIN 10
#define accYThreshold -0.5

WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid        = "iPhone";
const char* password    = "a0983711099";
const char* mqtt_server = "172.20.10.3";

//unsigned long myChannelNumber = 1850483; // บรรทัดที่ 142 ส่งค่าอุณหภูมิและความชื้น
//const char * myWriteAPIKey = "GC8AYRRONMOL9FWS"; 
const char * mqtt_username = "Natthanan";//1850483
const char * mqtt_password = "a0983711099";//a0983711099

// Google script ID and required credentials
String GOOGLE_SCRIPT_ID = "AKfycbzpi_iCwfolchZIMT9qmBeEzB9nz8kgh73f4NqJd4Fi7MIhPWM7rOZLxfN_MolJRTFG";    // change Gscript ID


void setupWifi();
void callback(char* topic, byte* payload, unsigned int length); // Mqtt
void reConnect();   // Mqtt

unsigned long now = 0;
unsigned long lastMsg = 0;
char msg[50];
char buf300[300];
int value = 0;
String messageTemp = "";

float x = 5.0F;
float y = 5.0F;
float z = 5.0F;

float gx = 5.0F;
float gy = 5.0F;
float gz = 5.0F;

float pitch = 5.0F;
float roll  = 5.0F;
float yaw   = 5.0F;

float tt = 50;

int stateServo = 0;
int stateServo2 = 0;
int stateaccY = 0;




Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
int servoNo = 0; // Defines a counter for servos. count0-n
int servoNoMax = 4; //maximum n servo

int adc4pin = 36; // If Wifi G0, G26 don't used
float volts0, volts1, volts2, volts3, volts4;
int servo0, servo1, servo2, servo3, servo4;

byte buf[2];

void setup() {
  Serial.begin(115200);
  //M5.begin();
  //M5.IMU.Init();  // Init IMU.
  //M5.Lcd.setRotation(3);
  //M5.Lcd.setCursor(40, 0);
  //M5.Lcd.println("IMU TEST");
  //M5.Lcd.setCursor(0, 10);
  //M5.Lcd.println("   X       Y       Z");
  //M5.Lcd.setCursor(0, 50);
  //M5.Lcd.println("  Pitch   Roll    Yaw");


  //Wire.begin(32, 33);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency


  pinMode(LEDPIN, OUTPUT);

  setupWifi();

  client.setServer(mqtt_server, 1883); // Sets the server details.
  client.setCallback(callback);  // Sets the message callback function.
  while (!client.connected()) {
      String client_id = "mqttx_f56baa16";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          client.subscribe("m5stickc_led");
          client.subscribe("teardown/fingers/grasp/");
          Serial.println("Public emqx mqtt broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
}


void loop() {

  //  Loop Mqtt
  if (!client.connected()) {
    reConnect();
  }
  client.loop();

  // Loop Main 1000ms
  now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;

    //M5.IMU.getGyroData(&gx, &gy, &gz);
    //M5.IMU.getAccelData(&x, &y, &z);
    //M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    //M5.IMU.getTempData(&tt);

    //M5.Lcd.setCursor(0, 20);
    //M5.Lcd.printf("%6.2f  %6.2f  %6.2f o/s\n", gx, gy, gz);        // rad/s
    //M5.Lcd.printf(" %5.2f   %5.2f   %5.2f G\n\n\n\n", x, y, z);    // 1g = 9.80665m/s2
    //M5.Lcd.printf(" %5.2f   %5.2f   %5.2f\n", pitch, roll, yaw);            // deg  between -180 and 180 deg
    //M5.Lcd.printf("Temperature : %.2f C", tt);

    x = random(-500, 500);
    y = random(-500, 500);
    z = random(-500, 500);
    tt = random(20, 30);
    volts0 = ADS1115_A0(0);
    volts1 = random(0, 33);
    volts2 = random(0, 33);
    volts3 = random(0, 33);
    volts4 = random(0, 33);
    

    Serial.printf("{\"x\" : %.3f,\"y\" : %.3f,\"z\" : %.3f, \"gyroX\" : %.3f, \"gyroY\" : %.3f,\"gyroZ\" : %.3f,\"temp\" : %.3f,\"volts0\" : %02.02f,\"volts1\" : %02.02f,\"volts2\" : %02.02f,\"volts3\" : %02.02f,\"volts4\" : %02.02f}", x/100, y/100, z/100, gx, gy, gz,tt,volts0,volts1/10,volts2/10,volts3/10,volts4/10);


    sprintf(buf300, "{\"x\" : %.3f,\"y\" : %.3f,\"z\" : %.3f, \"gyroX\" : %.3f, \"gyroY\" : %.3f,\"gyroZ\" : %.3f,\"temp\" : %.3f,\"volts0\" : %02.02f,\"volts1\" : %02.02f,\"volts2\" : %02.02f,\"volts3\" : %02.02f,\"volts4\" : %02.02f}", x/100, y/100, z/100, gx, gy, gz,tt,volts0,volts1/10,volts2/10,volts3/10,volts4/10);
    Serial.printf("%s\n", msg);

    client.publish("teardown/arm/move", buf300);

    if (WiFi.status() == WL_CONNECTED) {
    
    String urlFinal = "https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+"x=" + String(x/100) + "&y=" + String(y/100)+ "&z=" + String(z/100)+ "&volts0=" + String(volts0)+ "&volts1=" + String(volts1/10)+ "&temp=" + String(tt);
    Serial.print("POST data to spreadsheet:");
    Serial.println(urlFinal);
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET(); 
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;
    if (httpCode > 0) {
        payload = http.getString();
        Serial.println("Payload: "+payload);    
    }
    //---------------------------------------------------------------------
    http.end();
    Serial.println();
  }
  }

}

void setupWifi() {
  delay(10);
  //M5.Lcd.printf("Connecting to % s", ssid);
  Serial.printf("Connecting to % s\n", ssid);
  WiFi.mode(WIFI_STA);  // Set the mode to WiFi station mode.
  WiFi.begin(ssid, password);  // Start Wifi connection.

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //M5.Lcd.print(".");
    Serial.printf(".");
  }
  //M5.Lcd.printf("\nSuccess\n");
  Serial.printf("\nSuccess\n");
}

void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  messageTemp = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if (String(topic) == "m5stickc_led") {
    Serial.println("Changing output to ");

    if (messageTemp == "ON") {
      Serial.println("ON");
      digitalWrite(LEDPIN, LOW);

    }
    else if (messageTemp == "OFF") {
      Serial.println("OFF");
      digitalWrite(LEDPIN, HIGH);
    }
  }

  if (String(topic) == "teardown/fingers/grasp/") {
    Serial.println("Changing output to ");

    StaticJsonDocument <256> datamqtt;
    deserializeJson(datamqtt, payload);

    servo0 = datamqtt["servo0"];
    //servo1 = datamqtt["servo1"];
    //servo2 = datamqtt["servo2"];
    //servo3 = datamqtt["servo3"];
    //servo4 = datamqtt["servo4"];


    if (servo0 == 1) {
      // แบ
      Serial.printf("Relax\n");
      if (stateServo == 0) {
        stateServo = 1;
        controlServoRelax();
      }


    } else {
      // กำ
      Serial.printf("Contraction\n");
      if (stateServo == 1) {
        stateServo = 0;
        controlServoContraction();
      }
    }
  }
  Serial.println();
}

void reConnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID.
    String clientId = "M5Stack - ";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect.
    if (client.connect(clientId.c_str())) {
      client.subscribe("m5stickc_led");
      client.subscribe("teardown/fingers/grasp/");
      Serial.printf("\nSuccess\n");
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}


void controlServoRelax(void) {

  //for (int pulselen = 600; pulselen > 150; pulselen--) {
    pwm.setPWM(0, 0, angleToPulse(0));
    //pwm.setPWM(1, 0, pulselen);
    //pwm.setPWM(2, 0, pulselen);
    //pwm.setPWM(3, 0, pulselen);
    //pwm.setPWM(4, 0, pulselen);
  //}

}


void controlServoContraction(void) {

  //for (int pulselen = 150; pulselen < 600; pulselen++) {
    pwm.setPWM(0, 0, angleToPulse(180));
    //pwm.setPWM(1, 0, pulselen);
    //pwm.setPWM(2, 0, pulselen);
    //pwm.setPWM(3, 0, pulselen);
    //pwm.setPWM(4, 0, pulselen);
  //}

}

float ADS1115_A0(int channel) {

  /////////////////////////////////////////////////////
  // 1. Write to Config register (4byte)
  /////////////////////////////////////////////////////
  Wire.beginTransmission(0x48);   //Byte1  Slave Address
  Wire.write(0b00000001);         //Byte2  Choose Write Config Register
  //Byte3  MSB Bit 15 - 8 Config Register
  //Bit 12-14 A0:100, A1:101, A2:110, A3: 111
  if (channel == 0) {
    Wire.write(0b11000010);   //A0:100  Bit 12-14
  } else if (channel == 1) {
    Wire.write(0b11010010);   //A1:101  Bit 12-14
  } else if (channel == 2) {
    Wire.write(0b11100010);   //A2:110  Bit 12-14
  } else if (channel == 3) {
    Wire.write(0b11110010);   //A2:110  Bit 12-14
  }
  Wire.write(0b10000011);         //Byte4  LSB Bit 7  - 0 Config Register
  Wire.endTransmission();
  delay(20);

  /////////////////////////////////////////////////////
  // 2. Write to Address Pointer register (2byte)
  /////////////////////////////////////////////////////
  Wire.beginTransmission(0x48);   //Byte1  Slave Address
  Wire.write(0b00000000);         //Byte2  Choose Write Coversion Register
  Wire.endTransmission();
  delay(20);

  /////////////////////////////////////////////////////
  // 3. Read Conversion register (3byte)
  /////////////////////////////////////////////////////
  Wire.requestFrom(0x48, 2);  // Byte1 0x48<<0x01 ,  Byte2,3 read 2byte from Slave
  buf[0] = Wire.read();       // Byte2 Read MSB High Byte  (ADC Data)
  buf[1] = Wire.read();       // Byte3 Read LSB Low Byte  (ADC Data)
  //Wire.readBytes(buf, 2);   // Combo Read 2byte 
  uint16_t buf16 = (((0x0000 | buf[0]) << 8) | buf[1]);

  /////////////////////////////////////////////////////
  // 4. Covert ADC Data to Voltage
  /////////////////////////////////////////////////////
  //Filter Error 0xFFFF on GND
  if (buf16 == 0xffff) {
    buf16 = 0x0000;
  }
  float volt = buf16 * 0.000125; // 0.125mV / unit
  return volt;
}
