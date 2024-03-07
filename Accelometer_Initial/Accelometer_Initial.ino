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

// N.B. Replace with your calibration results
// For use when converting raw accelerometer values to m/s^@
#define ONE_G_DIFF (70) // ADC value difference between 0g and 1g
#define ZERO_G_POINT (347) // ADC value for 0g

// Moving Average Filter / Buffer / Track values
#define WINDOW_SIZE 5

//===========================================================================
// Initialising variables and peripherals
//===========================================================================

// Variables to hold acceleration readings in m/s^2
float accel_x, accel_y, accel_z;

void setup() {
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

  float new_x;
  float new_y;
  float new_z;

  
  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = raw_x;
  accel_y = raw_y;
  accel_z = raw_z;

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

  Serial.println(accel_z);

}