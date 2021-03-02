/*********************************************************************
 Bobble for Bluefruit nRF52832 v2
 has been created by derrick @ reachandteach.com and is a modified
 version of sample code for Adafruit's nRF52 bleuart sample code.
 The intent of this software is to create a simple preprogrammed controller
 to experiment with an attached motor or servo for controlling toys
 or other applications. Code has been added to drive a neopixel string

 Adafruit developed code has been provided under an MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution

 Any additional code by Reach and Teach is provided under a Creative Commons license allowing
 free use as long as attribution and share-alike licensing is respected.

 The software accepts and responds to single character inputs:
 '+' Sets MOTORENABLE (pin 7) HIGH 
 '-' Sets MOTORENABLE (pin 7) LOW
 '0' - '9' Drives pin 11 as a PWM servo output from 0 degrees to 180 degrees in 20 degree increments
 'F' Sets MOTOROUT1 (pin 16) HIGH and MOTOROUT2 (pin 15) LOW
 'R' Sets MOTOROUT2 (pin 16) LOW and MOTOROUT2 (pin 15) HIGH
 's' Sets servo to the angle passed on the stack
 'n' Sets and plays a peacetree neopixel pattern passed on the stack
 'N' Set a pattern of neopixel lights passed on the stack as <start> <stop> <color>
 'x' Reads an attached sensor voltage on A0
 '>' Sets high trigger value to last sensor reading
 '<' Sets low trigger value to last sensor reading
 '^' Sets low and high trigger for digital threshold
 ')' Sets command triggered by high trigger value to last command
 '(' Sets command triggered by low trigger value to last command
 '*' Auto trigger last command based on time passed on the stack
 '!' Enable sensor triggering
 '.' Disable sensor triggering and auto triggering
 
 Pins 7, 16, and 15 are designed to be connected to a L293DNE motor driver chip
 to drive a toy motor. Pin 7 will be connected to the EN chip enable pin. Pin 16
 and Pin 15 will be connected to the A chip pins. The motor will be connected to
 the Y chip pins.

 Pin A1/P0.03 is a connection point for a 50 element neopixel strand 

 Pin P0.11 is a connection point for a servo

 MOTORENABLE (pin 7) can also be used to control an external AC/DC Relay Outlet like
 an Adafruit Controllable Four Outlet Power Relay Module version 2

 A Chrome-based web application has been written to allow any BLE device to easily
 communicate with this application. Contact derrick@reachandteach.com for more info.
 
*********************************************************************/
#include <bluefruit.h>

#include <Servo.h>
#include "StackArray.h"
#include <Adafruit_NeoPixel.h>
#include <arduino-timer.h>

auto timer= timer_create_default();

#define DEVICENAME "RTBobble52_0004"
#define MOTORENABLE 7
#define MOTOROUT1 16
#define MOTOROUT2 15
#define SERVO 11
#define REDLED 17


#define DIGITALVLOW 400 // adc value for digital low
#define DIGITALVHIGH 700 // adc value for digital high

#define NSAMPLES 6 // number of sensor samples to take and average

// Reach and Teach code for servo control and sensor input
/************************* Servo Setup *************************************/

int servoPin = SERVO;  // Declare the Servo pin
int servopos = 0; // Servo value
Servo Servo1;  // Create a servo object
bool ServoOn = false;

int adcinx = A0; // assigned input for sensorCommand
int sensorx = 0; // last sensor reading
char lastcmd; // last bobble command stored for setting a trigger command
int lastneopixelp1= 0; // last neopixel string parm 1
int lastneopixelp2= 0; // last neopixel string parm 2

int triggerHighRef = DIGITALVHIGH; // Trigger command when sensor > this value
int triggerLowRef = DIGITALVLOW; // Trigger command when sensor < this value
char highTriggerCmd = '+'; // Command to be triggered by triggerHighRef
char lowTriggerCmd = '-'; // Command to be triggered by triggerLowRef
char autoTriggerCmd = '-'; // Command to be triggered by autotrigger
bool triggerEnable = false; // Sensor triggering enable
bool triggeredLow = false; // Ensure that we only trigger once
bool triggeredHigh = false; // Ensure that we only trigger once
bool hilocmd = true; //toggle starting with high autotrigger, alternating with low autotrigger

int parms[6]={0,0,0,0,0,0}, high_parms[6], low_parms[6]; //cmd parms

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// create a stack of integers.
StackArray <int> stack;

// setup neopixels
#define PIN 3 //A1 Neopixel strand
#define N_LEDS 50 // number of neopixels in strand
int peaceValue= 0;
int colorsel = 0;
int cred, cgreen, cblue, cbluegreen, cyellow;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// Default light peace pattern for peace tree. Used for bobble values < 2000
static void lightPeacePattern() {
  if (peaceValue % 1000 > 1) {
    cred = min(peaceValue * 3, 128);
  } else {
    cred = 0;
  }
  if (peaceValue % 1000 > 10) {
    cgreen = min(peaceValue * 3, 128);
  } else {
    cgreen = 0;
  }
  if (peaceValue % 1000 > 20) {
    cblue = min(peaceValue * 3, 128);
  } else {
    cblue = 0;
  }
  if (peaceValue % 1000 > 30) {
    cbluegreen = min(peaceValue * 3, 128);
  } else {
    cbluegreen = 0;
  }
  if (peaceValue % 1000 > 35) {
    cyellow = min(peaceValue * 3, 128);
  } else {
    cyellow = 0;
  }

  chase(strip.Color(cred, 0, 0), (peaceValue >= 1000));
  chase(strip.Color(0, cgreen, 0), (peaceValue >= 1000));
  chase(strip.Color(0, 0, cblue), (peaceValue >= 1000));
  chase(strip.Color(0, cbluegreen, cbluegreen), (peaceValue >= 1000));
  chase(strip.Color(cyellow, cyellow, 0), (peaceValue >= 1000));

}

// Light Pattern 2. Useful for progress/fundraising. Used for bobble values between 2000 and <3000.
static void lightPattern2() {
  strip.fill(0, 0, N_LEDS);
  for (int i=0; i<N_LEDS; i++){
    if (i > 0) {
      strip.setPixelColor(i-1, 0, 0, 0);
    }
    if (i >= peaceValue-2000){
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      if (i % 20 < 4) {
        strip.setPixelColor(i, 128, 0, 0);
      } 
      if (i % 20 >= 4 && i % 20 < 8) {
        strip.setPixelColor(i, 0, 128, 0);
      } 
      if (i % 20 >= 8 && i % 20 < 12) {
        strip.setPixelColor(i, 0, 0, 128);
      } 
      if (i % 20 >= 12 && i % 20 < 16 ) {
        strip.setPixelColor(i, 0, 128, 128);
      } 
      if (i % 20 >= 16 && i % 20 < 20) {
        strip.setPixelColor(i, 128, 128, 0);
      } 
      
    }
    strip.show();
    delay(50);
  }

}

// Light Pattern 3. Useful for progress/fundraising. Used for bobble values between 3000 and <4000.
static void lightPattern3() {
  strip.fill(0, 0, N_LEDS);
  strip.show();
  delay(200);
  for (int i=0; i<N_LEDS; i++){
    if (i >= peaceValue-3000){
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      if (i % 20 < 4) {
        strip.setPixelColor(i, 128, 0, 0);
      } 
      if (i % 20 >= 4 && i % 20 < 8) {
        strip.setPixelColor(i, 0, 128, 0);
      } 
      if (i % 20 >= 8 && i % 20 < 12) {
        strip.setPixelColor(i, 0, 0, 128);
      } 
      if (i % 20 >= 12 && i % 20 < 16 ) {
        strip.setPixelColor(i, 0, 128, 128);
      } 
      if (i % 20 >= 16 && i % 20 < 20) {
        strip.setPixelColor(i, 128, 128, 0);
      } 
      
    }
    strip.show();
    delay(50);
  }

}

static void lightPattern4() {
  int cred= int(peaceValue % 4000 / 100)*20;
  int cgreen= int((peaceValue % 4000 / 10) % 10)*20;
  int cblue= int((peaceValue % 4000) % 10)*20;

  chase(strip.Color(cred, cgreen, cblue), false);
}

static void lightPattern5(int nstart, int nstop, int ncolor) {
  int cred= int(ncolor / 100)*20;
  int cgreen= int((ncolor / 10) % 10)*20;
  int cblue= int(ncolor % 10)*20;

  for (int i=nstart; i<= nstop; i++){
    strip.setPixelColor(i, cred, cgreen, cblue);
  }
  strip.show();
}


static void chase(uint32_t c, bool twinkle) {
  for (uint16_t i = 0; i < strip.numPixels() + 4; i++) {
    strip.setPixelColor(i  , c); // Draw new pixel
    strip.setPixelColor(i - 4, 0); // Erase pixel a few steps back
    if (twinkle) {
      strip.setPixelColor(random(0, strip.numPixels() - 1), strip.Color(96, 96, 96));
    } 
    strip.show();
    delay(25);
  }
}

void peacetree(){
    //Display neopixel string per peacetree algorithm
  if (peaceValue < 2000){
    lightPeacePattern();
  }
  if (peaceValue >=2000 && peaceValue < 3000){
    lightPattern2();
  }
  if (peaceValue >=3000 && peaceValue < 4000){
    lightPattern3();
  }
  if (peaceValue >=4000 && peaceValue < 5000){
    lightPattern4();
  }
}

int readSensor(){
  int reading = 0;
  for (int i=0; i<NSAMPLES; i++){
    reading += analogRead(adcinx);
  }
  return reading/NSAMPLES; 
}


void driveBobble(char c, bool triggering=false){
      switch (c) {
      case '+': // motor enable on
        digitalWrite(REDLED, HIGH);
        digitalWrite(MOTORENABLE, HIGH);
        Serial.print(" [motoron] ");
        lastcmd = c;
        delay(300);
        break;
      case '-': // motor enable off
        digitalWrite(REDLED, LOW);
        digitalWrite(MOTORENABLE, LOW);
        Serial.print(" [motoroff] ");
        lastcmd = c;
        delay(300);
        break;
      case 'F': // motor forward direction
        digitalWrite(MOTOROUT1,HIGH);
        digitalWrite(MOTOROUT2,LOW);
        Serial.print(" [motorforward] ");
        lastcmd = c;
        break;
      case 'R': // motor reverse direction
        digitalWrite(MOTOROUT1,LOW);
        digitalWrite(MOTOROUT2,HIGH);
        Serial.print(" [motorreverse] ");
        lastcmd = c;
        break;
      case 's': // set servo
        if (!triggering && !stack.isEmpty()) parms[0]= stack.pop();
        servopos= parms[0];
        Serial.print (" [servo position "); Serial.print (servopos); Serial.print ("] ");
        lastcmd = c;
        break;
      case 'n': // set neopixel pattern
        if (!triggering && !stack.isEmpty()) parms[0]= stack.pop();
        peaceValue= parms[0];
        Serial.print (" [peaceValue "); Serial.print (peaceValue); Serial.print ("] ");
        peacetree();
        lastcmd = c;
        break;
      case 'N': // set neopixel pattern
        if (!triggering) {
          if (!stack.isEmpty()) parms[2]= stack.pop();
          if (!stack.isEmpty()) parms[1]= stack.pop();
          if (!stack.isEmpty()) parms[0]= stack.pop();
        }
        Serial.print (" [set neopixels "); 
        Serial.print (parms[0]); Serial.print (", ");
        Serial.print (parms[1]); Serial.print (", ");
        Serial.print (parms[2]);
        Serial.print ("] ");
        lastcmd = c;

        if (parms[1]>parms[0]) {
          lightPattern5(parms[0], parms[1], parms[2]);
        }
        break;
      case 'x': // read sensor x
        sensorx = readSensor();
        Serial.print(" [sensorx="); 
        Serial.print(sensorx); 
        Serial.print("] ");
        bleuart.println(sensorx);
        break;
      case '>': // set high trigger value to last sensor reading
        triggerHighRef = sensorx;
        Serial.print(" [set high trigger =");
        Serial.print(sensorx); 
        Serial.print("] ");
        break;
      case '<': // set low trigger value to last sensor reading
        triggerLowRef = sensorx;
        Serial.print(" [set low trigger =");
        Serial.print(sensorx); 
        Serial.print("] ");
        break;
      case '^': // set low and high trigger for digital threshold
        triggerLowRef = DIGITALVLOW;
        triggerHighRef = DIGITALVHIGH;
        Serial.print(" [set digital trigger] ");
        break;
      case ')': // set high trigger command
        highTriggerCmd = lastcmd;
        for (int i=0; i<6; i++) high_parms[i]= parms[i];
        Serial.print(" [high trigger command =");
        Serial.print(lastcmd); 
        Serial.print("] ");
        break;
      case '(': // set low trigger command
        lowTriggerCmd = lastcmd;
        for (int i=0; i<6; i++) low_parms[i]= parms[i];
        Serial.print(" [low trigger command =");
        Serial.print(lastcmd); 
        Serial.print("] ");
        break;
      case '*': // enable autotrigger
        if (!triggering && !stack.isEmpty()) {
          autoTriggerCmd = lastcmd;
          int interval= stack.pop();
          timer.every(interval, autotriggercall);
          Serial.print(" [autotrigger command=");
          Serial.print(lastcmd);
          Serial.print(" interval=");
          Serial.print(interval);
          Serial.print("] ");
        }
        break;
      case '!': // enable sensor triggering
        triggerEnable = true;
        Serial.print(" [enable trigger] ");
        break;
      case '.': // disable sensor triggering and autotriggering
        triggerEnable = false;
        timer.cancel(); // cancel autotriggering if necessary
        Serial.print(" [disable trigger] ");
        break;              
        
      default: // see if it is a number, if so get the rest of the number until NAN
        if (c>=48 && c<=57){
          int ninput= c-48;
          // get the rest of the number
          int ch1;
          while ((ch1 = bleuart.read()) != EOF && ch1 != '\n')
          {
            if (ch1>=48 && ch1<=57){
              Serial.write(ch1);
              ninput= ninput*10+(ch1-48);
            } else {
              break;
            }
          }
          stack.push(ninput);
        }
        break;
    }
    
}


void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("Reach and Teach BLE Bobble v2.0");
  Serial.println("-------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICENAME);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  // Reach and Teach code starts here
  pinMode(REDLED, OUTPUT);
  digitalWrite(REDLED, LOW);      // turn Red LED off
  pinMode(MOTOROUT1, OUTPUT);  // motor direction 1
  pinMode(MOTOROUT2, OUTPUT);  // motor direction 2
  pinMode(MOTORENABLE, OUTPUT);  // motor enable
  digitalWrite(MOTORENABLE, LOW);
  digitalWrite(MOTOROUT1, HIGH);
  digitalWrite(MOTOROUT2, LOW);
  Servo1.attach(servoPin); // Attach servo
  ServoOn=true;

  Serial.print("MOTORENABLE="); Serial.println(MOTORENABLE);
  Serial.print("MOTOROUT1="); Serial.println(MOTOROUT1);
  Serial.print("MOTOROUT2="); Serial.println(MOTOROUT2);
  Serial.print("SERVO="); Serial.println(SERVO);
  Serial.println("ADCIN0=A0");
  Serial.println("NEOPIXEL PIN=A1/P0.03");
  

}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);
  
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet

  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void highTriggerCall(){
  for (int i=0; i<6; i++) parms[i]= high_parms[i];
  driveBobble(highTriggerCmd,true);
  Serial.println("");
}

void lowTriggerCall(){
  for (int i=0; i<6; i++) parms[i]= low_parms[i];
  driveBobble(lowTriggerCmd,true);
  Serial.println("");
}

bool autotriggercall (void *argument) {
  if (hilocmd){
    highTriggerCall();
  } else {
    lowTriggerCall();
  }
  hilocmd= !hilocmd;  
  return true; // to repeat the action - false to stop
}


void loop()
{
  timer.tick(); // tick the timer
  // Forward data from HW Serial to BLEUART
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
  }

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
    driveBobble(ch); // bobble output control
    Serial.println("");
  }
  if (ServoOn) Servo1.write(servopos); // Drive servo based on servo on/off

  if (triggerEnable) { // if enabled we should be taking sensor readings to trigger cmd
    sensorx = readSensor();
    if (sensorx > triggerHighRef && !triggeredHigh ) { //Trigger command if sensor is over high trigger ref
      triggeredLow = false;
      triggeredHigh = true;
      highTriggerCall();
    }
    if (sensorx < triggerLowRef && !triggeredLow) {  //Trigger command if sensor is under low trigger ref
      triggeredLow = true;
      triggeredHigh = false;
      lowTriggerCall();
    }    
  }

}
