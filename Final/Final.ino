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

// Declare Functions - To be called in Loop Code
// Read from Accelerometer.
void readFromAccelerometer(){
  // Read raw values from accelerometer on the analog pins
  int raw_x = analogRead(A1);
  int raw_y = analogRead(A2);
  int raw_z = analogRead(A3);

  // Convert from raw accelerometer values to m/s^2 using pre-calibrated values
  //TODO: Properly calibrate - current values only from z testing 
  float new_x=map(raw_x,292,438,-g,g);
  float new_y=map(raw_y,285,432,-g,g);
  float new_z=map(raw_z,302,442,-g,g);

  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = new_x;
  accel_y = new_y;
  accel_z = new_z;

}

//TODO: Actually use or lose this
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

//TODO: MAKE THIS WORK PROPERLY
float calcVelocity(char direction_char, float acceleration, unsigned long diff_time){
  float velocity = 0; 
  float prev_x, prev_y, prev_z=0; //TODO: Write a new switch statment to set prev values to first value when initializing 
  int intcheck=0;
  bool check=false; 

  //Convert diff_time to seconds 
  float current_time = static_cast<float>(diff_time) / 1000;
  //This static cast is neccesary, as I was getting persistent issues with the conversion without it
  
  //v = u + at
  switch(direction_char) {
    case 'x':
    velocity = prev_x + acceleration*current_time;
    prev_x=velocity-prev_x;
    break; 

    case 'y':
    velocity = prev_y + acceleration*current_time;
    prev_y=velocity-prev_y;
    break;

    case 'z':
    velocity = prev_z + acceleration*current_time;
    prev_z=velocity;
    break; 

    default:
    Serial.println("ERROR: Invalid direction char passed to calcVelocity");
    break;
  }

  return velocity;
}

float calcAltitude(float temperature, float pressure){
  //N.B Constants declared at top of file - 
  float virt_temp=temperature+273.15;
  float altitude = ((R * virt_temp) / g) * log(P0 / pressure);
  return altitude;
}

float calcForce(float acceleration){
  //F = m*a
  float force = ROCKET_MASS*acceleration;

  //Returns force in Newtons
  return force;
}

int detectMode(int prev_mode){
  int mode_code; 
  if (prev_mode == 0) {
    mode = PRE_LAUNCH;
    prev_mode = PRE_LAUNCH;
  }

  else if(prev_mode == PRE_LAUNCH && curr_alt > 10) {
    mode = ASCENDING;
    prev_mode = ASCENDING;
  }

  else if (prev_mode == ASCENDING && curr_alt < max_alt) {
    mode = DESCENDING;
    prev_mode = DESCENDING;
  }

  else if (prev_mode == DESCENDING && curr_alt == min_alt) {
    mode = LANDED;
    prev_mode = LANDED;
  }
  return mode_code;
  
}

//===========================================================================
// TRACK CALCULATED READINGS 
//===========================================================================

void trackAltitude(int time_now, float alt_curr){
  int min_alt_time, max_alt_time = 0; 
  bool initial_alt = false;
  
  //Initialiaze the max/min with first read value
  if(initial_alt==false) {
    min_alt, max_alt = alt_curr;
    initial_alt=true;
  }

  //Check if current value is new maximum
  if(alt_curr>max_alt) {
    max_alt_time = time_now;
    max_alt = alt_curr;
  }

  //Check if current value is new minimum 
  if(alt_curr<min_alt) {
    min_alt_time = time_now;
    min_alt = alt_curr;
  }
}

void trackAccelX(int time_now){
// record min, max, avg, acceleration for X
  int min_accel_x_time, max_accel_x_time, avg_accel_x = 0; 
  bool initial_accel_x = false;
  
  //Initialiaze the max/min with first read value
  if(initial_accel_x==false) {
    min_accel_x, max_accel_x = accel_x;
    initial_accel_x=true;
  }

  //Check if current value is new maximum
  if(accel_x>max_accel_x) {
    max_accel_x_time = time_now;
    max_accel_x = accel_x;
  }

  //Check if current value is new minimum 
  if(accel_x<min_accel_x) {
    min_accel_x_time = time_now;
    min_accel_x = accel_x;
  }

  //Get simple average 
  avg_accel_x = (max_accel_x+min_accel_x)/2;
}

void trackAccelY(int time_now){
// record min, max, avg, acceleration for Y
int min_accel_y_time, max_accel_y_time, avg_accel_y = 0; 
  bool initial_accel_y = false;
  
  //Initialiaze the max/min with first read value
  if(initial_accel_y==false) {
    min_accel_y, max_accel_y = accel_y;
    initial_accel_y=true;
  }

  //Check if current value is new maximum
  if(accel_y>max_accel_y) {
    max_accel_y_time = time_now;
    max_accel_y = accel_y;
  }

  //Check if current value is new minimum 
  if(accel_y<min_accel_y) {
    min_accel_y_time = time_now;
    min_accel_y = accel_y;
  }

  //Get simple average 
  avg_accel_y = (max_accel_y+min_accel_y)/2;
}

void trackAccelZ(int time_now){
// record min, max, avg, acceleration for Z
int min_accel_z_time, max_accel_z_time, avg_accel_z = 0; 
  bool initial_accel_z = false;
  
  //Initialiaze the max/min with first read value
  if(initial_accel_z==false) {
    min_accel_z, max_accel_z = accel_z;
    initial_accel_z=true;
  }

  //Check if current value is new maximum
  if(accel_z>max_accel_z) {
    max_accel_z_time = time_now;
    max_accel_z = accel_z;
  }

  //Check if current value is new minimum 
  if(accel_z<min_accel_z) {
    min_accel_z_time = time_now;
    min_accel_z = accel_z;
  }

  //Get simple average 
  avg_accel_z = (max_accel_z+min_accel_z)/2;
}

void trackVelX(int time_now){
// record min, max, avg, velocity for X
  int min_vel_x_time, max_vel_x_time, avg_vel_x = 0; 
  bool initial_vel_x = false;
  
  //Initialiaze the max/min with first read value
  if(initial_vel_x==false) {
    min_vel_x, max_vel_x = vel_x;
    initial_vel_x=true;
  }

  //Check if current value is new maximum
  if(vel_x>max_vel_x) {
    max_vel_x_time = time_now;
    max_vel_x = vel_x;
  }

  //Check if current value is new minimum 
  if(vel_x<min_vel_x) {
    min_vel_x_time = time_now;
    min_vel_x = vel_x;
  }

  //Get simple average 
  avg_vel_x = (max_vel_x+min_vel_x)/2;
}

void trackVelY(int time_now){
// record min, max, avg, velocity for Y
int min_vel_y_time, max_vel_y_time, avg_vel_y = 0; 
  bool initial_vel_y = false;
  
  //Initialiaze the max/min with first read value
  if(initial_vel_y==false) {
    min_vel_y, max_vel_y = vel_y;
    initial_vel_y=true;
  }

  //Check if current value is new maximum
  if(vel_y>max_vel_y) {
    max_vel_y_time = time_now;
    max_vel_y = vel_y;
  }

  //Check if current value is new minimum 
  if(vel_y<min_vel_y) {
    min_vel_y_time = time_now;
    min_vel_y = vel_y;
  }

  //Get simple average 
  avg_vel_y = (max_vel_y+min_vel_y)/2;
}

void trackVelZ(int time_now){
// record min, max, avg, velocity for Z
int min_vel_z_time, max_vel_z_time, avg_vel_z = 0; 
  bool initial_vel_z = false;
  
  //Initialiaze the max/min with first read value
  if(initial_vel_z==false) {
    min_vel_z, max_vel_z = vel_z;
    initial_vel_z=true;
  }

  //Check if current value is new maximum
  if(vel_z>max_vel_z) {
    max_vel_z_time = time_now;
    max_vel_z = vel_z;
  }

  //Check if current value is new minimum 
  if(vel_z<min_vel_z) {
    min_vel_z_time = time_now;
    min_vel_z = vel_z;
  }

  //Get simple average 
  avg_vel_z = (max_vel_z+min_vel_z)/2;
}

void trackForceX(int time_now){
// record min, max, avg, force for X
  int min_force_x_time, max_force_x_time, avg_force_x = 0; 
  bool initial_force_x = false;
  
  //Initialiaze the max/min with first read value
  if(initial_force_x==false) {
    min_force_x, max_force_x = force_x;
    initial_force_x=true;
  }

  //Check if current value is new maximum
  if(force_x>max_force_x) {
    max_force_x_time = time_now;
    max_force_x = force_x;
  }

  //Check if current value is new minimum 
  if(force_x<min_force_x) {
    min_force_x_time = time_now;
    min_force_x = force_x;
  }

  //Get simple average 
  avg_force_x = (max_force_x+min_force_x)/2;
}

void trackForceY(int time_now){
// record min, max, avg, force for Y
int min_force_y_time, max_force_y_time, avg_force_y = 0; 
  bool initial_force_y = false;
  
  //Initialiaze the max/min with first read value
  if(initial_force_y==false) {
    min_force_y, max_force_y = force_y;
    initial_force_y=true;
  }

  //Check if current value is new maximum
  if(force_y>max_force_y) {
    max_force_y_time = time_now;
    max_force_y = force_y;
  }

  //Check if current value is new minimum 
  if(force_y<min_force_y) {
    min_force_y_time = time_now;
    min_force_y = force_y;
  }

  //Get simple average 
  avg_force_y = (max_force_y+min_force_y)/2;
}

void trackForceZ(int time_now){
// record min, max, avg, force for Z
int min_force_z_time, max_force_z_time, avg_force_z = 0; 
  bool initial_force_z = false;
  
  //Initialiaze the max/min with first read value
  if(initial_force_z==false) {
    min_force_z, max_force_z = force_z;
    initial_force_z=true;
  }

  //Check if current value is new maximum
  if(force_z>max_force_z) {
    max_force_z_time = time_now;
    max_force_z = force_z;
  }

  //Check if current value is new minimum 
  if(force_z<min_force_z) {
    min_force_z_time = time_now;
    min_force_z = force_z;
  }

  //Get simple average 
  avg_force_z = (max_force_z+min_force_z)/2;
}

//===========================================================================

void transmit(String data_to_send){
  // Send data to PC for debugging
  Serial.println(data_to_send);
  
  // Send to HC-12 for wireless transmission
  HC12.println(data_to_send);

  delay(1);
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

  // 2. RUN CALCULATIONS ON SENSOR DATA

  // Calculate time variables
  time_diff = millis() - time_prev;
  time_prev = millis() - time_start;
  
  // Calculate Altitude
  curr_alt = calcAltitude(TEMP_AVERAGED, PRESSURE_AVERAGED);

  // Calculate velocity for each axis using calcVelocity.
  vel_x = calcVelocity('x', X_AVERAGED, time_diff);
  vel_y = calcVelocity('y', Y_AVERAGED, time_diff);
  vel_z = calcVelocity('z', Z_AVERAGED, time_diff);

  // Calculate Force (in Newtons)
  force_x = calcForce(X_AVERAGED);
  force_y = calcForce(Y_AVERAGED);
  force_z = calcForce(Z_AVERAGED);

  // Detect Mode i.e. launch, ascending, descending, landed etc.
  int mode_code = detectMode(prev_mode);

  // TRACK MIN, MAX, AVG VALUES for Altitude, Acceleration, Force, and Velocity. 
  trackAltitude(time_prev, curr_alt); 
  
  trackAccelX(time_prev);
  trackAccelY(time_prev);
  trackAccelZ(time_prev);

  trackVelX(time_prev);
  trackVelY(time_prev);
  trackVelZ(time_prev);

  trackForceX(time_prev);
  trackForceY(time_prev);
  trackForceZ(time_prev);



  // GENERATE TRANSMISSION STRING
  String data_to_send = "";
  
  //Switch statement for different modes goes HERE
   switch(mode_code){
    case PRE_LAUNCH:
      // Only transmit once every 5s, use some kind of timer.
      if ((time_prev - pre_launch_last_transmit_time) > 5){
        data_to_send = data_to_send + "," + curr_alt + "," + accel_y;
        data_to_send = data_to_send + "5 seconds since last PRE_LAUNCH transmit - transmit since start: " + time_prev;
        Serial.println(data_to_send);
        // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
        //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n"; 

        // Transmit data
        transmit(data_to_send);

        // Record PRE_LAUNCH transmission time
        pre_launch_last_transmit_time = time_prev;
      }
      else{
        Serial.println(F("5 seconds have not passed since previous PRE_LAUNCH transmit. Wait."));
      }
      break;
      
    case ASCENDING:
      // Transmit as fast as possible
      // Package data into a string
        data_to_send = data_to_send + "," + curr_alt + "," + accel_x + "," + accel_y + "," + accel_z + "," + temperature + "," + pressure;
        Serial.println(data_to_send);
      // FORMAT: ALTITUDE, ACCEL (X,Y,Z), TEMPERATURE, PRESSURE
      
      // Transmit data
      transmit(data_to_send);
      break;
      
    case DESCENDING:
      // Transmit as fast as possible
      // Package data into a string
        data_to_send = data_to_send + "," + curr_alt + "," + accel_x + "," + accel_y + "," + accel_z + "," + temperature + "," + pressure;
        Serial.println(data_to_send);
      // FORMAT: ALTITUDE, ACCEL (X,Y,Z), TEMPERATURE, PRESSURE
      // Transmit data
      transmit(data_to_send);
      break;

    case LANDED:
      if ((time_prev - landed_last_transmit_time) > 10000) {
        // Provide summary e.g. of maximum values
        data_to_send = data_to_send + "Summary of results" + "\n";
        data_to_send = data_to_send + "Maximum altitude: " + max_alt + "\n";
        data_to_send = data_to_send + "Maximum upward acceleration: " + max_accel_y + "\n";
        data_to_send = data_to_send + "Maximum downward: " + min_accel_y + "\n";
        data_to_send = data_to_send + "Time at landing: " + time_prev + "\n";
        HC12.println(data_to_send);
      }
      break;

    default:
      break;
  }
  /* Debug Check
  String raw_debug = "";
  raw_debug = raw_debug + time_prev;
  transmit(raw_debug);
  */

  /* OLD TRANSMIT CODE - OUTSIDE OF MODES - REMOVE IN FINAL ITERATION
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
     data_to_send = data_to_send + X_AVERAGED + "," + Y_AVERAGED + "," + Z_AVERAGED + "," + TEMP_AVERAGED + "," + PRESSURE_AVERAGED + "," + curr_alt;
    //data_to_send = data_to_send + vel_x + "," + vel_y + "," + vel_z;
  }
  */ //REMOVE ^^^^^^^^^^^^^^^^^^^^
  
  transmit(data_to_send);
  delay(100);
}