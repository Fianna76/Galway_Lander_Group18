/***************************************************************************
  Uni. of Galway - Modified Lander Program Template
  Authors: 
    - Oisín McDonnell Killilea

    - o.mcdonnellkillilea1@universityofgalway.ie
================================
***************************************************************************/

//===========================================================================
// Import necessary libraries
//===========================================================================

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

//===========================================================================
// Constants - These never change
//===========================================================================

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x60) //0x60 for BME and 0x58 for BMP.

// For use when converting raw accelerometer values to m/s^2
#define ONE_G_DIFF (140) // ADC value difference between 0g and 1g
#define ZERO_G_POINT (442) // ADC value for 0g
#define g (9.81) // Value for gravity

// Moving Average Filter / Buffer / Track values
#define WINDOW_SIZE 5

//===========================================================================
// Initialising variables and peripherals
//===========================================================================

// Variables to hold acceleration readings in m/s^2
float accel_x, accel_y, accel_z;

// Moving average filter variables
int INDEX = 0;
int VALUE[3] = {0};
int SUM[3] = {0};
int X_READINGS[WINDOW_SIZE]={0};
int Y_READINGS[WINDOW_SIZE]={0};
int Z_READINGS[WINDOW_SIZE]={0};
int TEMP_READINGS[WINDOW_SIZE];
int PRESSURE_READINGS[WINDOW_SIZE];
int X_AVERAGED = 0;
int Y_AVERAGED = 0;
int Z_AVERAGED = 0;
int TEMP_AVERAGED = 0;
int PRESSURE_AVERAGED = 0;

void setup() {
  // Code has started, start timestamp.
  //time_start = millis();
  
  // Setup serial connections to the HC-12/ PC
  // Start serial streams between Arduino and HC-12/ PC
  //HC12.begin(9600);
  Serial.begin(9600);

  // Give console time to get running
  while(!Serial){
    Serial.println(F("Waiting for serial connection..."));
  }
}
// Declare Functions - To be called in Loop Code
// Read from Accelerometer.
void readFromAccelerometer(){
  // Read raw values from accelerometer on the analog pins
  int raw_x = analogRead(A1);
  int raw_y = analogRead(A2);
  int raw_z = analogRead(A3);

  // Convert from raw accelerometer values to m/s^2 using pre-calibrated values
  //TODO: Properly calibrate - current values only from z testing 
  float new_x=map(raw_x,302,442,-g,g);
  float new_y=map(raw_y,302,442,-g,g);
  float new_z=map(raw_z,302,442,-g,g);

  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = new_x;
  accel_y = new_y;
  accel_z = new_z;

}

int average_array(int arrayToSum[], int sizeOfArray){
  int sum, average = 0;
  for ( int i = 0; i < sizeOfArray; i++ ){
    sum += arrayToSum[ i ];
  }

  // divide by array size to get average 
  average = sum / sizeOfArray;
  
  return average;
}

void smoothAccelReading(){
  //TODO: Comment properly, and see if you can refactor this to work without having to change the global variables from the example code 
  //TODO: They GAVE us an average_array function - USE IT
  //I'm 99% sure they did not intend for me to use arrays like this here
  //This is a crime against nature and god will judge me for my sins
  
  int temp[3];

  //Removes the oldest entries from the sum
  temp[0]=SUM[0]-X_READINGS[INDEX];
  temp[1]=SUM[1]-Y_READINGS[INDEX];
  temp[2]=SUM[2]-Z_READINGS[INDEX];
  
  //Store all the axis readings as items in an array
  VALUE[0] = accel_x;
  VALUE[1] = accel_y;
  VALUE[2] = accel_z;

  //Add each reading to the window 
  X_READINGS[INDEX] = VALUE[0];  
  Y_READINGS[INDEX] = VALUE[1];
  Z_READINGS[INDEX] = VALUE[2];

  //Add the newest readings to the SUM
  for(int i=0;i<3;i++) {
    SUM[i] = temp[i] + VALUE[i];
    /*DEBUG CODE: Archived
    Serial.println(i);
    Serial.print("=Sum=");
    Serial.println(SUM[i]);
    Serial.print("=Value=");
    Serial.println(VALUE[i]);
    Serial.print("=Temp=");
    Serial.println(temp[i]);
    Serial.print("=Index=");
    Serial.println(INDEX);
    delay(2000);
    */
  }
    /* DEBUG CODE: Archived
    Serial.print("=X=");
    Serial.println(X_READINGS[INDEX]);
    Serial.print("=Y=");
    Serial.println(Y_READINGS[INDEX]);
    Serial.print("=Z=");
    Serial.println(Z_READINGS[INDEX]);
    */
    

  //Stores the average                  
  X_AVERAGED = SUM[0] / WINDOW_SIZE;
  Y_AVERAGED = SUM[1] / WINDOW_SIZE;
  Z_AVERAGED = SUM[2] / WINDOW_SIZE;

  
}

float calcVelocity(char direction_char, float acceleration, unsigned long diff_time){
  // TODO: Add parameters and functionality

  float velocity = 0; 
  float prev_velocity;

  // HINT: Use Newton's SUVAT equations
  return velocity;
}



void loop() {
  // 1. READ FROM SENSORS.



  // Read Accelerometer and then Smooth
  readFromAccelerometer();
  smoothAccelReading();

  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  // GENERATE TRANSMISSION STRING
  String data_to_send = "";
  // Use commas to separate values to generate transmssion string
  data_to_send = String(X_AVERAGED) + "," + String(Y_AVERAGED) + "," + String(Z_AVERAGED) + ",";
  //data_to_send = String(accel_x) + "," + String(accel_y) + "," + String(accel_z) + ",";


  Serial.println(data_to_send);
  delay(100);
  //Serial.println(accel_x);
  //Serial.println(accel_y);
  //Serial.println(accel_z);

}