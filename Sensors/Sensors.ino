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

// Bosch Environmental Sensor (BME/BMP280)
Adafruit_BMP280 env_sensor; // use I2C interface
Adafruit_Sensor *env_temp = env_sensor.getTemperatureSensor();
Adafruit_Sensor *env_pressure = env_sensor.getPressureSensor();

// Variables to hold acceleration readings in m/s^2
float accel_x, accel_y, accel_z;

// Variable to hold the environmental sensor readings (degrees C & hPa)
float temperature, pressure;

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
  
  //Temporary array - technically not needed, fixed a bug early in development
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

void loop() {
  // 1. READ FROM SENSORS.



  // Read Accelerometer and then Smooth
  readFromAccelerometer();
  smoothAccelReading();

  // Read Environmental Sensor and then Smooth
  readEnvironmental();
  smoothEnvSensorReading();

  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  // GENERATE TRANSMISSION STRING
  String data_to_send = "";
  data_to_send = String(X_AVERAGED) + "," + String(Y_AVERAGED) + "," + String(Z_AVERAGED) + "," + String(TEMP_AVERAGED) + "," + String(PRESSURE_AVERAGED);
  Serial.println(data_to_send);
  
  /* Debug Check
  String raw_debug = "";
  raw_debug = String(accel_x) + "," + String(accel_y) + "," + String(accel_z) + "," + String(temperature) + "," + String(pressure);
  Serial.println(raw_debug);
  */

  delay(100); 
}