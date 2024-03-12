/***************************************************************************
  Uni. of Galway - Modified Lander Program Template
  Authors: 
    - Oisín McDonnell Killilea

    - o.mcdonnellkillilea1@universityofgalway.ie
================================
***************************************************************************/


//===========================================================================
// Constants - These never change
//===========================================================================

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x60) //0x60 for BME and 0x58 for BMP.

// Constants for use in code
#define SEALEVELPRESSURE_HPA (1013.25) // Pressure at sea level in hPa
#define ROCKET_MASS (0.2) // Mass of rocket/capsule in kilograms


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

void setup() {
  // Code has started, start timestamp.
  time_start = millis();
  
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
  // HINT: Look at the slides, use the map() function



  float new_x=map(raw_x,302,442,-g,g);
  float new_y=map(raw_x,302,442,-g,g);
  float new_z=map(raw_z,302,442,-g,g);

  
  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = new_x;
  accel_y = new_y;
  accel_z = new_z;

}


void smoothAccelReading(){
  // Useful Example: https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter
  // Steps:
  // 1. index tells us which value we are replacing 
  // 2. Read in next Values
  // 3. calculate sum of array
  // 4. Store the averaged/smoothed sensor value into global variable

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

  //Serial.println(accel_x);
  Serial.println(accel_y);
  //Serial.println(accel_z);

}