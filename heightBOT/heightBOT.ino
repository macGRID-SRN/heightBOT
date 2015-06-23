#include "Servo.h"
#define servoPin 3
#include <I2C.h>
#include<math.h>

// Global Variables
int servoPosition;
int distance;
int lastDistance;
int totalAngle;
int horizontalDistance;
Servo myServo; 
bool stopMeasuring; 
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f    
  
void setup(){ 
  myServo.attach(servoPin);
  servoPosition = 0;
  lastDistance = 0;
  stopMeasuring = false;
  myServo.write(servoPosition);
  delay(500);
  Serial.begin(9600);
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  Serial.println("END SETUP==============");
  horizontalDistance = llGetDistanceAverage(9);
}

void loop(){
  lastDistance = distance;
  distance = llGetDistanceAverage(9);
  Serial.println(distance);
  if(atMaxDistance()){
    totalAngle = totalAngle - 5;
    moveServoDegrees(-5);
    stopMeasuring = true;
  }
  else{
    totalAngle = totalAngle + 5;
    moveServoDegrees(5); 
  }
}


int llGetDistanceAverage(int numberOfReadings){ 
  if(numberOfReadings < 2){
    numberOfReadings = 2; // If the number of readings to be taken is less than 2, default to 2 readings
  }
  int sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetDistance(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}

void moveServoDegrees(int n){
  if(!stopMeasuring){
  servoPosition = servoPosition + n;
  myServo.write(servoPosition);
  delay(300);
  }else{
    Serial.println("Distance = ");
    Serial.println(getHeight(distance));
    delay(5000);
  }
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

int llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return(distance);
}

int GetDistance(){
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
  return distance;
}

double getDistanceChange(int first, int second){
  //Serial.println("Actual change=======" + String((first * first) + (second * second) - (2 * first* second * cos( (PI * 5) / 180))) );
 return sqrt((first * first) + (second * second) - (2 * first* second * cos( (PI * 5) / 180)));
} 

double getExpectedDistanceChange(int lastDistance){
 //Serial.println("Expected Change=======" + String((horizontalDistance / cos((totalAngle * PI) / 180)) - sqrt((horizontalDistance * horizontalDistance) + (lastDistance * lastDistance)) ));
  return  (horizontalDistance / cos((totalAngle * PI) / 180)) - sqrt((horizontalDistance * horizontalDistance) + (lastDistance * lastDistance));
}

double getHeight(int distance){
  return sqrt((distance * distance) - (horizontalDistance * horizontalDistance));
}
