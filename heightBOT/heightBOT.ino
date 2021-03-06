#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

// These are the pins used for the UNO
// for Due/Mega/Leonardo use the hardware SPI pins (which are different)
#define _sclk 13
#define _miso 12
#define _mosi 11
#define _cs 10
#define _dc 9
#define _rst 8

#include "Servo.h"
#define servoPin 3
#include <I2C.h>
#include<math.h>

// Global Variables
int servoPosition;
double distance;
double lastDistance;
int totalAngle;
double horizontalDistance;
Servo myServo; 
bool stopMeasuring;
int numOver = 0;

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f    

Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);

int dispX = 320;
int dispY = 240;

int lastY = 1; //The next Y to print at on the TFT

void setup(){ 
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9340_BLACK);
  tft.setTextColor(ILI9340_RED);  tft.setTextSize(2);
  
  myServo.attach(servoPin);
  servoPosition = 15;
  lastDistance = 0;
  stopMeasuring = false;
  myServo.write(servoPosition);
  delay(500);
  Serial.begin(9600);
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  Serial.println("END SETUP==============");
  
  tft.setCursor(1,lastY);
  tft.println("END SETUP======");
  
  horizontalDistance = llGetDistanceAverage(20) / 100.0;
}

void loop(){

  if(stopMeasuring){
    Serial.println("Distance = ");
    Serial.println(getHeight(distance));
    
    tftPrintLn(1, "Distance = ");
    tftPrint(130, (String) getHeight(distance));
    
    tftPrintLn(1, "END MEASURE======");
    tftPrintLn(1, "=========================");
        
    myServo.write(25);
    
    delay(50000000);
  }
  
  int saveDist = lastDistance;
  lastDistance = distance;
  distance = llGetDistanceAverage(9);
  Serial.println(distance);

  if(overshoot(distance)){
    lastDistance = saveDist;
    if(numOver > 1){
      stopMeasuring = true;
      distance = lastDistance;
    }else{
    totalAngle = totalAngle - 25;
    moveServoDegrees(-25);
    Serial.println("Overshoot detected. Remeasuring.");
    
    tftPrintLn(1, "Overshoot detected. Retry.");
    
    }
  } 
  else if(atMaxDistance()){
    totalAngle = totalAngle - 5;
    moveServoDegrees(-5);
    stopMeasuring = true;
    Serial.println("Max Value Detected");
    
    tftPrintLn(1, "Max Value Detected");
  }
  else{
    totalAngle = totalAngle + 5;
    moveServoDegrees(5);
    delay(250);
  }
}

bool overshoot(int distance){
  Serial.println("Checking Overshoot ========");
  double oldHeight = getHeight(lastDistance);
  Serial.print("Calculated Height: "); 
  Serial.println(oldHeight);
  double expectedHeight = getEstHeight();
  Serial.print("Estimated Height: "); 
  Serial.println(expectedHeight);
  
  tftPrintLn(1, "=========================");
  tftPrintLn(1, "Checking Overshoot ======");
  tftPrintLn(1, "Calc Height: ");
  tftPrint(150, (String) oldHeight);
  tftPrintLn(1, "Est. Height: ");
  tftPrint(150, (String) expectedHeight); 
  
  if(distance/expectedHeight > 1.3)
  {
    numOver++;
    return true;
  }
  else return false;
}

double getEstHeight(){
  Serial.print("Old angle: ");
  double oldAngle = getEstAngle(lastDistance);
  Serial.println(oldAngle);
  Serial.print("New angle: ");
  double nextAngle = oldAngle + getActualDegree(5);
  Serial.println(nextAngle);
  
  tftPrintLn(1, "Old A: ");
  tftPrint(80, (String) oldAngle);
  tftPrint(150, "New A: ");
  tftPrint(230, (String) nextAngle); 
  
  return horizontalDistance / cos((nextAngle/ 180)* PI);
}

double getEstAngle(int a){
  double angle = acos((double)horizontalDistance/(double)a)/ PI * 180;
  return angle;
}

double getHeight(double distance){
  return sqrt((distance * distance) - (horizontalDistance * horizontalDistance));
}

double getActualDegree(int degree){
  return 55*((double)degree)/145;
}

double llGetDistanceAverage(int numberOfReadings){ 
  if(numberOfReadings < 2){
    numberOfReadings = 2; // If the number of readings to be taken is less than 2, default to 2 readings
  }
  double sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetDistance(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}

void moveServoDegrees(int n){
  servoPosition = servoPosition + n;
  myServo.write(servoPosition);
  delay(300);
}

bool atMaxDistance(){
  return distance < 1;
}

// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2]; // Return array for use in other functions
}



/* ==========================================================================================================================================
Get 2-byte distance from sensor and combine into single 16-bit int
=============================================================================================================================================*/

double llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return((double)distance);
}

double GetDistance(){
    uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
  // Print Distance
  return (double)distance;
}

//Try to print to the TFT, after we get past or to max Y we will refresh the screen
void tftPrintLn(int X, String str) {
  lastY += 16;
  
  if (lastY >= dispY) {
    lastY = 1;
    //tft.fillScreen(ILI9340_BLACK);  
    //tft.setTextColor(ILI9340_RED);  tft.setTextSize(2);
    delay(500); //Wait a tiny bit for the values to be read
    
  }
  
  tft.fillRect(1, lastY, dispX, 32, ILI9340_BLACK);
  
  tft.setCursor(X,lastY);
  tft.println(str);
}

//Try to print to the TFT at the current lastY value
void tftPrint(int X, String str) {
  tft.setCursor(X,lastY);
  tft.println(str);
}
