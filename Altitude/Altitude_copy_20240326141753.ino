/***************************************************************************
  Uni. of Galway - Modified Lander Program Template
  Authors: 
    - Oisín McDonnell Killilea
    - Roshan George
    - Ethan Delaney
  Email:
    - r.george5@universityofgalway.ie
    - e.delaney11@universityofgalway.ie
    - o.mcdonnellkillilea1@universityofgalway.ie
    
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

// HC-12 Wireless Transceiver
SoftwareSerial HC12(7, 6); // HC-12 TX Pin, HC-12 RX Pin

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x60) //0x60 for BME and 0x58 for BMP.

// Mode Codes
#define PRE_LAUNCH (100)
#define ASCENDING (200)
#define DESCENDING (300)
#define LANDED (400)

// For use when converting raw accelerometer values to m/s^2
#define ONE_G_DIFF (140) // ADC value difference between 0g and 1g
#define ZERO_G_POINT (442) // ADC value for 0g
#define g (9.81) // Value for gravity

// Moving Average Filter / Buffer / Track values
#define WINDOW_SIZE 5

// Constants for use in code
#define ROCKET_MASS (0.2) // Mass of rocket/capsule in kilograms

const float R = 287; // Universal gas constant in J/(kg·K))
const float T0 = 288.15; // Standard temperature at sea level in Kelvin
const float P0 = 989; // Standard pressure at campus ground level in Hectopascals

//===========================================================================
// Initialising variables and peripherals
//===========================================================================

// Bosch Environmental Sensor (BME/BMP280)
Adafruit_BMP280 env_sensor; // use I2C interface
Adafruit_Sensor *env_temp = env_sensor.getTemperatureSensor();
Adafruit_Sensor *env_pressure = env_sensor.getPressureSensor();

// Variables to hold acceleration readings in m/s^2
float accel_x, accel_y, accel_z;

// Variable to hold the environmental sensor readings (degrees C & hPa)
float temperature, pressure;

//Variable to hold the altitude calcuation (m)
float curr_alt;

// time variables
unsigned long time_start, time_prev, time_diff, pre_launch_last_transmit_time, landed_last_transmit_time = 0;

//mode variables
int prev_mode, mode = 0;

// Moving average filter variables
int INDEX = 0;
int VALUE[5] = {0};
int SUM[5] = {0};
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

// calcuated variables
float prev_velocity_x, prev_velocity_y, prev_velocity_z = 0;
float vel_x, vel_y, vel_z = 0;
float force_x, force_y, force_z= 0;

// min/max variables
float min_temp, max_temp, min_pressure, max_pressure, min_alt, max_alt = 0;
float min_vel_x, max_vel_x, min_vel_y, max_vel_y, min_vel_z, max_vel_z = 0;
float min_accel_x, max_accel_x, min_accel_y, max_accel_y, min_accel_z, max_accel_z = 0;
float min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z = 0;

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

  int temp[2]={0};

  //Removes the oldest entries from the sum
  temp[0]=SUM[3]-TEMP_READINGS[INDEX];
  temp[1]=SUM[4]-PRESSURE_READINGS[INDEX];
  
  // Read in next Values
  VALUE[3]=temperature;
  VALUE[4]=pressure;

  //Add readings to the window
  TEMP_READINGS[INDEX]=VALUE[3];
  PRESSURE_READINGS[INDEX]=VALUE[4];

  //Better ways of doing this - external function it, see comments in accelerometerSmoothing
  for(int i=3;i<=4;i++) {
    SUM[i] = temp[i-3] + VALUE[i];
    /*DEBUG CODE
    Serial.println("==|i|==");
    Serial.println(i);
    Serial.println("==|SUM|==");
    Serial.println(SUM[i]);
    Serial.println("==|VALUE|==");
    Serial.println(VALUE[i]);
    Serial.println("==|Temp|==");
    Serial.println(temp[i-3]);
    delay(1000);
    */
  }

  // Store the averaged/smoothed sensor value into global variable
  TEMP_AVERAGED=SUM[3] / WINDOW_SIZE;
  PRESSURE_AVERAGED=SUM[4] / WINDOW_SIZE;
}



float calcAltitude(float temperature, float pressure){
  //N.B Constants declared at top of file - 
  float virt_temp=temperature+273.15;
  float altitude = ((R * virt_temp) / g) * log(P0 / pressure);
  return altitude;
}


void transmit(String data_to_send){
  // Send data to PC for debugging
  Serial.println(data_to_send);
  
  // Send to HC-12 for wireless transmission
  HC12.println(data_to_send);

  delay(1);
}

void loop() {
  // 1. READ FROM SENSORS.

  
  // Read Environmental Sensor and then Smooth
  readEnvironmental();
  smoothEnvSensorReading();

  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  // 2. RUN CALCULATIONS ON SENSOR DATA

  // Calculate time variables
  time_diff = millis() - time_prev;
  time_prev = millis() - time_start;
  
  // Calculate Altitude
  curr_alt = calcAltitude(TEMP_AVERAGED, PRESSURE_AVERAGED);

 


  // GENERATE TRANSMISSION STRING
  String data_to_send = "";
 
  /* Debug Check
  String raw_debug = "";
  raw_debug = raw_debug + time_prev;
  transmit(raw_debug);
  */

  // OLD TRANSMIT CODE - OUTSIDE OF MODES - REMOVE IN FINAL ITERATION
  if(time_prev!=time_prev) {
        data_to_send = data_to_send + "Summary of results" + "\n";
        data_to_send = data_to_send + "Maximum altitude: " + max_alt + "\n";
        //TODO: Determine orientation for "up"
        data_to_send = data_to_send + "Maximum upward acceleration: " + max_accel_y + "\n";
        data_to_send = data_to_send + "Maximum downward: " + min_accel_y + "\n";

        //TODO: Add proper summary
  }
  else
  {
     data_to_send = data_to_send + curr_alt + "," + temperature + "," + pressure;
    //data_to_send = data_to_send + vel_x + "," + vel_y + "," + vel_z;
  }
  // //REMOVE ^^^^^^^^^^^^^^^^^^^^
  
  transmit(data_to_send);
  delay(100);
}