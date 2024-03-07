/***************************************************************************
  Uni. of Galway - Full Lander Program Template
  Authors: 
    - Roshan George
    - Ethan Delaney
  Email:
    - r.george5@universityofgalway.ie
    - e.delaney11@universityofgalway.ie
================================
***************************************************************************/

//===========================================================================
// Import necessary libraries
//===========================================================================

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//===========================================================================
// Constants - These never change
//===========================================================================

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x60) //0x60 for BME and 0x58 for BMP.

// Mode Codes
#define PRE_LAUNCH (100)
#define ASCENDING (200)
#define DESCENDING (300)
#define LANDED (400)

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

// HC-12 Wireless Transceiver
SoftwareSerial HC12(7, 6); // HC-12 TX Pin, HC-12 RX Pin

// Bosch Environmental Sensor (BME/BMP280)
Adafruit_BMP280 env_sensor; // use I2C interface
Adafruit_Sensor *env_temp = env_sensor.getTemperatureSensor();
Adafruit_Sensor *env_pressure = env_sensor.getPressureSensor();

// Variables to hold acceleration readings in m/s^2
float accel_x, accel_y, accel_z;

// Variable to hold the environmental sensor readings (degrees C & hPa)
float temperature, pressure;

// time variables
unsigned long time_start, time_prev, time_diff, pre_launch_last_transmit_time, landed_last_transmit_time = 0;

//mode variable
int prev_mode, mode = 0;

// Moving average filter variables
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int X_READINGS[WINDOW_SIZE];
int Y_READINGS[WINDOW_SIZE];
int Z_READINGS[WINDOW_SIZE];
int TEMP_READINGS[WINDOW_SIZE];
int PRESSURE_READINGS[WINDOW_SIZE];
int X_AVERAGED = 0;
int Y_AVERAGED = 0;
int Z_AVERAGED = 0;
int TEMP_AVERAGED = 0;
int PRESSURE_AVERAGED = 0;

// calcuated variables
float prev_velocity_x, prev_velocity_y, prev_velocity_z = 0;
float vel_x, vel_y, vel_z = 0;
float force_x, force_y, force_z= 0;

// min/max variables
float min_temp, max_temp, min_pressure, max_pressure, min_alt, max_alt = 0;
float min_vel_x, max_vel_x, min_vel_y, max_vel_y, min_vel_z, max_vel_z = 0;
float min_accel_x, max_accel_x, min_accel_y, max_accel_y, min_accel_z, max_accel_z = 0;
float min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z = 0;
//unsigned long min_temp_tp, max_temp_tp ... minforce_y_tp;

//===========================================================================
// Setup code - Housekeeping code
//===========================================================================

void setup() {
  // Code has started, start timestamp.
  time_start = millis();

  // Setup serial connections to the HC-12/ PC
  // Start serial streams between Arduino and HC-12/ PC
  HC12.begin(9600);
  Serial.begin(9600);

  // Give console time to get running
  while(!Serial){
    Serial.println(F("Waiting for serial connection..."));
  }

  // Setup for the Environmental Sensor
  if (!env_sensor.begin(I2CADDR, CHIPID)) {
    Serial.println(F("Could not find a valid BME/BMP280 (Environmental) sensor, check CHIPID/libraries/wiring!"));
    while (1) delay(10);
  }

  // Default Adafruit_BMP280 settings from datasheet/
  env_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                         Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                         Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                         Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                         Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  // initialise times 
  pre_launch_last_transmit_time = time_start;
  landed_last_transmit_time = time_start;
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
  // Useful Example: https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter
  // Steps:
  // 1. index tells us which value we are replacing 
  // 2. Read in next Values
  // 3. calculate sum of array
  // 4. Store the averaged/smoothed sensor value into global variable

}

void readEnvironmental(){
  // Read Environmental Sensor
  sensors_event_t temp_event, pressure_event;
  
  env_temp->getEvent(&temp_event);
  env_pressure->getEvent(&pressure_event);
  
  // Store value into variables: temperature (Celcius), pressure (hPa)
  temperature = temp_event.temperature;
  pressure = pressure_event.pressure;
}

void smoothEnvSensorReading(){
  // Apply filtering or smoothing to temperature (Celcius) and pressure (hPa)
  // index tells us which value we are replacing 

  // Read in next Values

  // calculate sum of array

  // Store the averaged/smoothed sensor value into global variable

}
// RUN CALCULATIONS ON SENSOR DATA

float calcAltitude(float temperature, float pressure){
  // TODO for students: Add parameters and functionality.
  float altitude;

  // HINT: Use the Hypsometric formula
  return altitude;
}

float calcVelocity(char direction_char, float acceleration, unsigned long diff_time){
  // TODO: Add parameters and functionality

  float velocity = 0; 
  float prev_velocity;

  // HINT: Use Newton's SUVAT equations
  return velocity;
}

float calcForce(){
  // TODO for students: Add parameters and functionality.
  float force;

  // HINT: Use the rocket/capsule mass
  return force;
}

int detectMode(int prev_mode)
{
  // TODO for students: Add parameters and functionality.

  // new mode
  int mode_code;

  /* HINT: 
   *  - Which sensor values and/or calculated values can be used here?
   *  - How will these values look throughout the launch?
   *  - How will you track these changes to know when to switch mode?
   *  - COnsider which mode the mode which you are previously in - this should dictat/constrain whichi mode you can go move into next (tip: draw it out as a 'state machine' diagram and put it into your report)
   */

  return mode_code;
}

// TRACK CALCULATED READINGS 
void trackAltitude(int time_now){
// TODO for students: Add parameters and functionality.
}

void trackVelX(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, velocity for X
}

void trackVelY(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, velocity for Y
}

void trackVelZ(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, velocity for Z
}

void trackForceX(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, force for x
}

void trackForceY(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, force for y
}

void trackForceZ(int time_now){
// TODO for students: Add parameters and functionality.
// record min, max, avg, force for z
}

void transmit(String data_to_send){
  // Send data to PC for debugging
  Serial.println(data_to_send);
  
  // Send to HC-12 for wireless transmission
  HC12.println(data_to_send);

  delay(1);
}

// Loop Code - Main functionality
void loop() 
{
  // 1. READ FROM SENSORS.

  // Get current timestamp
    float time_now = millis();

  // TODO Get difference between current timestamp and previous timestamp for use in calculations later.

  // Read Accelerometer and then Smooth
  readFromAccelerometer();
  smoothAccelReading();

  // Read Environmental Sensor and then Smooth
  readEnvironmental();
  smoothEnvSensorReading();

  // After finsihed smoothing, increment index
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  // 2. RUN CALCULATIONS ON SENSOR DATA
  
  // Calculate Altitude
  float curr_alt = calcAltitude(temperature, pressure);
  // Calculate velocity for each axis using calcVelocity.
  vel_x = calcVelocity('x', accel_x, time_diff);
  vel_y = calcVelocity('y', accel_y, time_diff);
  vel_z = calcVelocity('z', accel_z, time_diff);
  
  // Calculate Force
  // HINT: Use the rocket/capsule mass
  force_x = calcForce();
  force_y = calcForce();
  force_z = calcForce();

  // Detect Mode i.e. launch, ascending, descending, landed etc.
  // this can go here... or alterantively, if you wish to detect mode based off raw values, then put this above the calucalted values
  int mode_code = detectMode(prev_mode);

  // TRACK MIN, MAX, AVG VALUES for Altitude, Force, and Velocity. 
  trackAltitude(time_now);
  
  trackVelX(time_now);
  trackVelY(time_now);
  trackVelZ(time_now);

  trackForceX(time_now);
  trackForceY(time_now);
  trackForceZ(time_now);

  // GENERATE TRANSMISSION STRING

  String data_to_send = "";
  // Use commas to separate values to generate transmssion string
  // data_to_send = value1 + "," + value2 + "," + value3 + ","....

  //Depending on mode - run certain calculations/ transmit particular data at different intervals
  switch(mode_code){
    case PRE_LAUNCH:
      // Only transmit once every 5s, use some kind of timer.
      if ((time_now - pre_launch_last_transmit_time) > 5){
        data_to_send = "";
        data_to_send = data_to_send + "5 seconds since last PRE_LAUNCH transmit - transmit since start: " + time_now;
        Serial.println(data_to_send);
        // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
        //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n"; 

        // Transmit data
        transmit(data_to_send);

        // Record PRE_LAUNCH transmission time
        pre_launch_last_transmit_time = time_now;
      }
      else{
        Serial.println(F("5 seconds have not passed since previous PRE_LAUNCH transmit. Wait."));
      }
      break;
      
    case ASCENDING:
      // Transmit as fast as possible
      // Package data into a string
      // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
      //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n";  

      // Transmit data
      transmit(data_to_send);
      break;
      
    case DESCENDING:
      // Transmit as fast as possible
      // Package data into a string
      // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
      //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n"; 
      // Transmit data
      transmit(data_to_send);
      break;

    case LANDED:
      if ((time_now - landed_last_transmit_time) > 10000) {
        // Provide summary e.g. of maximum values
        data_to_send = data_to_send + "Summary of results" + "\n";
        data_to_send = data_to_send + "Maximum altitude: " + max_alt + "\n";
        data_to_send = data_to_send + "Maximum upward velocity: " + max_vel_x + "\n";
        data_to_send = data_to_send + "Maximum downward: " + min_vel_x + "\n";
        HC12.println(data_to_send);
        // Transmit other statistics...
        HC12.println(data_to_send);
        // Transmit other statistics...
        HC12.println(data_to_send);
        landed_last_transmit_time = time_now;
      }
      break;

    default:
      break;
  }
}
