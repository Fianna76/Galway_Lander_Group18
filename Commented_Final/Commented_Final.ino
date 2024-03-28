/***************************************************************************
  Uni. of Galway - Modified Lander Program Template
  Authors: 
    - Oisín McDonnell Killilea
    - James Mullarkey
    - Roshan George
    - Ethan Delaney
  Email:
    - o.mcdonnellkillilea1@universityofgalway.ie
    - INSERT YOUR EMAIL HERE 
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

// HC-12 Wireless Transceiver
SoftwareSerial HC12(7, 6); // HC-12 TX Pin, HC-12 RX Pin

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x58) //0x60 for BME and 0x58 for BMP.

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
#define ROCKET_MASS (0.3) // Mass of rocket/capsule in kilograms
#define GROUND_LEVEL (10) //Ground level as detected by sensors
#define DESCENT_CHECK (15) //The amount of consecutive sensor reads to detect if the rockets descending 
//N.B SET ABOVE CONSTANTS TO CORRECT VALUES BEFORE LAUNCH

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
unsigned long mode_time[4] = {0}; //0 - Pre-Launch, 1 - Ascending, 2 - Descending, 3 - Landing

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
float min_temp, max_temp, avg_temp, min_pressure, max_pressure, avg_pressure, min_alt, max_alt, avg_alt = 0;
float min_vel_x, max_vel_x, min_vel_y, max_vel_y, min_vel_z, max_vel_z, avg_vel_x, avg_vel_y, avg_vel_z = 0;
float min_accel_x, max_accel_x, avg_accel_x, min_accel_y, avg_accel_y, max_accel_y, min_accel_z, max_accel_z, avg_accel_z = 0;
float min_force_x, max_force_x, avg_force_x, min_force_y, max_force_y, avg_force_y, min_force_z, max_force_z, avg_force_z= 0;

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

// Inputs: None | Returns: None | Stores acceleration values from the sensor into global variables accel_[DIRECTION], in ms^2
void readFromAccelerometer(){
  // Read raw values from accelerometer on the analog pins
  int raw_x = analogRead(A1);
  int raw_y = analogRead(A2);
  int raw_z = analogRead(A3);

  // Convert from raw accelerometer values to m/s^2 using pre-calibrated values
  float new_x=map(raw_x,292,438,-g,g);
  float new_y=map(raw_y,285,432,-g,g);
  float new_z=map(raw_z,302,442,-g,g);

  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = new_x;
  accel_y = new_y;
  accel_z = new_z;
}

// Inputs: None | Returns: None | Smooths the global acceleration values and stores in global AVERAGED_[DIRECTION] variables
void smoothAccelReading(){

  //Temporary array - technically not needed, fixed a bug early in development
  //Ensures that we don't get weird issues from our arrays without declaring them as static
  int temp[3];

  //Removes the oldest entries from the sum
  //This does nothing on the first iteration, but after we average the amount of values in INDEX, 
  //the filter moves upwards through the latest INDEX number of values
  temp[0]=SUM[0]-X_READINGS[INDEX];
  temp[1]=SUM[1]-Y_READINGS[INDEX];
  temp[2]=SUM[2]-Z_READINGS[INDEX];
  
  //Store all the axis readings as items in an array
  VALUE[0] = accel_x;
  VALUE[1] = accel_y;
  VALUE[2] = accel_z;

  //Add each reading to the window to be averaged
  X_READINGS[INDEX] = VALUE[0];  
  Y_READINGS[INDEX] = VALUE[1];
  Z_READINGS[INDEX] = VALUE[2];

  //Add the newest readings to the SUM 
  //The temp array should equal the last value of SUM sans the oldest read value
  //The VALUE array adds the newest read value to the SUM
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
    

  //Stores the average into global variables                  
  X_AVERAGED = SUM[0] / WINDOW_SIZE;
  Y_AVERAGED = SUM[1] / WINDOW_SIZE;
  Z_AVERAGED = SUM[2] / WINDOW_SIZE;
}

// Inputs: None | Returns: None | Stores sensor data to global variables temperature & pressure in Celsius and Hectopascals
void readEnvironmental(){
  // Read Environmental Sensor
  sensors_event_t temp_event, pressure_event;
  
  //Built in sensor functions
  env_temp->getEvent(&temp_event);
  env_pressure->getEvent(&pressure_event);
  
  // Store value into variables: temperature (Celcius), pressure (hPa)
  temperature = temp_event.temperature;
  pressure = pressure_event.pressure;
}

// Inputs: None | Returns: None | Smooths the global temperature and pressure values and stores them in global [data]_AVERAGED variables
void smoothEnvSensorReading(){
  //This function works on the same basis as the smoothAccelReading, and they both use the same global arrays for SUM & VALUE
  //The intention was to eventually combine both smoothing functions into one, but these functions worked, and we prioritised writing new functions
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

// Inputs: Char [X/Y/Z], float [Smoothed Accelation], unsigned long [Time interval since last call] | Returns: float [Calculated directional velocity] 
// Calculates the velocity in each axis according to Newtons SUVAT equations. Intergrated across time by using the change in time as t 
float calcVelocity(char direction_char, float acceleration, unsigned long diff_time){
  float velocity = 0; 
  float prev_x, prev_y, prev_z=0; //If the sensor is not at rest the loop starts, these being set at 0 will introduce a small delay before the correct velocity is returned

  float adjusted_accel = acceleration-9; //Adjusted acceleration value for Y axis only to account for gravity - additional comments below

  //Convert diff_time to seconds 
  float current_time = static_cast<float>(diff_time) / 1000;
  //This static cast is neccesary, there were persistent issues with the conversion without it
  
  //v = u + at
  //u = prev_[DIRECTION] (m/s)
  //a = acceleration (m/s^2)
  //t = current time (seconds)
  switch(direction_char) {
    case 'x':
    velocity = prev_x + acceleration*current_time; //Calculate current velocity
    prev_x=velocity-prev_x; //Set previous velocity for next iteration 
    break; 

    /*For y - since the sensor is placed vertically "up" along the y axis, we must account for gravity which would otherwise persistently generate a velocity#
    even while the sensor remained at rest. The other axis are fine without this as they shouldn't be affected by gravity unless the rocket goes sideways */
    case 'y':
    velocity = prev_y + adjusted_accel*current_time; //Calculate current velocity
    prev_y=velocity-prev_y; //Set previous velocity for next iteration 
    break;

    case 'z':
    velocity = prev_z + acceleration*current_time; //Calculate current velocity
    prev_z=velocity; //Set previous velocity for next iteration 
    break; 

    default:
    Serial.println("ERROR: Invalid direction char passed to calcVelocity");
    break;
  }

  //Return calculated velocity - our velocity is actually stored in global variables, but each direction is correctly defined in the loop to ease calculation
  return velocity;
}

// Inputs: floats [smoothed temperature/pressure] | Returns: Current Altitude (metres) | Calculates current altitude using Hypsometric Equation from lecture slides
float calcAltitude(float temperature, float pressure){
  //N.B Constants declared at top of file - 

  //Formula is seperated into multiple lines to make it more human readible and easier to debug
  float power = (1 / 5.257); //The value the two pressure variables are put to the power of
  float pressure_calc = pow((P0 / pressure),power); //Calculating left most side of numerator
  float altitude = ((pressure_calc-1) * (temperature + 273.15))/0.0065; //Rest of Equations calculations performed here

  //Note - altitude is stored in global variable curr_alt - alt_curr is a local variable used in a later external function only
  return altitude;
}

// Inputs: float [smoothed acceleration] | Returns: None | Performs simple force calculation for each axis
float calcForce(float acceleration){
  //F = m*a
  float force = ROCKET_MASS*acceleration; //Note ROCKET_MASS must be updated at top of code to the rockets final mass

  //Returns force in Newtons
  return force;
}

// Inputs: int [Previous Mode Value] int [Current time] | Returns: None | Tracks if the current mode should change, and tracks the time of each mode change
int detectMode(int prev_mode, int time){ 
  //Note - mode changes shouldn't really be a big thing for the live transmission within the final test - but we do have to track the times of each mode change
  int Falling=0; //This variable iterates for each time the rockets current altitude has been below its maximum altitude - used for descending mode change
  bool initial=false; 

  //Sets the initial mode to PRE_LAUNCH the first time this function is called, then never iterates again
  if (initial ==false) {
    mode = 0;
    prev_mode = 0;

    mode_time[0]=time_prev;
    initial=true;
  }

  //Sets the current mode to ASCENDING if the current alitude exceeds the pre-defined ground level altitude
  if(prev_mode == 0 && curr_alt > GROUND_LEVEL) {
    
    mode = 1;
    prev_mode = 1;

    mode_time[1]=time_prev;
    }
  }

  //Sets the current mode to descending once the current altitude has been below the maximum recorded altitude for several concurrent iterations
  else if (prev_mode == 1 && curr_alt < max_alt) {
    Falling++; //Increases Falling each time this is true - once the Falling variable exceeds the pre-defined DESCENT_CHECK the mode changes to DESCENDING
    if(Falling>=DESCENT_CHECK) {
    mode = 2;
    prev_mode = 2;
    }

    mode_time[2]=time_prev;
  }

  //Once the current altitude has reached the minimum recorded altitude the capsule has logically hit the ground again, and can move to LANDED 
  else if (prev_mode == 2 && curr_alt == min_alt) {
    mode = 3;
    prev_mode = 3;

    mode_time[3]=time_prev;
  }

  //Return the mode based on above criteria
  return mode; 
}

//===========================================================================
// TRACK CALCULATED READINGS 
//===========================================================================

// Inputs: int [current time], float [variable to be tracked], float [previous maximum] | Returns: float [maximum recorded value for variable] 
// Tracks the maximum values for each variable simply by comparing the current value to the previously recorded maximum, and saving it if it is higher
float trackMax(int time_now, float input, float max){
  int max_time = 0; //The max_time variable was added to be able to display the time each value reached its maximum, but was never actually used
  //Check if current value is new maximum, returns the new maximum if it is
  if(input>max) {
    max_time = time_now;
    max = input; 
    return(max);
  }
}

// Inputs: int [current time], float [variable to be tracked], float [previous minimum] | Returns: float [minimum recorded value for variable] 
// Tracks the minimum values for each variable simply by comparing the current value to the previously recorded minimum, and saving it if it is lower
float trackMin(int time_now, float input, float min){
  int min_time = 0; //The min_time variable was added to be able to display the time each value reached its minimum, but was never actually used

  //Check if current value is new minimum, returns the new minimum if it is 
  if(input<min) {
    min_time = time_now;
    min = input;
  }
  return(min);
}

//===========================================================================

//Inputs: String [Data to be transmited] (CSV format) | Returns: None | Transmits data across both Serial and Wireless transmitter
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

  // This was commented out because we believed the mode changes were causing our issues in our final launch test - so we attempted to force it straight into the final landed mode
  int mode_code;
  // Detect Mode i.e. launch, ascending, descending, landed etc.
  /*int mode_code = detectMode(prev_mode, time_prev);
  if(time_prev>5) {
     mode_code = 3;
  }*/

  // TRACK MIN, MAX, AVG VALUES
  //Average values are calculated here in the loop, and stored as global variables - they technically could be local but it's easier to declare max, min and avg in one place

  //Temperature:
  max_temp = trackMax(time_prev, TEMP_AVERAGED, max_temp);
  min_temp = trackMin(time_prev, TEMP_AVERAGED, min_temp);
  avg_temp = (max_temp+min_temp)/2;

  //Pressure:
  max_pressure = trackMax(time_prev, PRESSURE_AVERAGED, max_pressure);
  min_pressure = trackMin(time_prev, PRESSURE_AVERAGED, min_pressure);
  avg_pressure = (max_pressure+min_pressure)/2;

  //Altitude:
  max_alt = trackMax(time_prev, curr_alt, max_alt); 
  min_alt = trackMin(time_prev, curr_alt, min_alt); 
  avg_alt = (max_alt+min_alt)/2;

  //Acceleration:
  //X axis:
  max_accel_x = trackMax(time_prev, X_AVERAGED, max_accel_x);
  min_accel_x = trackMin(time_prev, X_AVERAGED, min_accel_x);
  avg_accel_x = (max_accel_x+min_accel_x)/2;
  //Y axis:
  max_accel_y = trackMax(time_prev, Y_AVERAGED, max_accel_y);
  min_accel_y = trackMin(time_prev, Y_AVERAGED, min_accel_y);
  avg_accel_y = (max_accel_y+min_accel_y)/2;
  //Z axis:
  max_accel_z = trackMax(time_prev, Z_AVERAGED, max_accel_z);
  min_accel_z = trackMin(time_prev, Z_AVERAGED, min_accel_z);
  avg_accel_z = (max_accel_z+min_accel_z)/2;

  //Velocity
  //X axis:
  max_vel_x = trackMax(time_prev, vel_x, max_vel_x);
  min_vel_x = trackMin(time_prev, vel_x, min_vel_x);
  avg_vel_x = (max_vel_x+min_vel_x)/2;
  //Y axis:
  max_vel_y = trackMax(time_prev, vel_y, max_vel_y);
  min_vel_y = trackMin(time_prev, vel_y, min_vel_y);
  avg_vel_y = (max_vel_y+min_vel_y)/2;
  //Z axis:
  max_vel_z = trackMax(time_prev, vel_z, max_vel_z);
  min_vel_z = trackMin(time_prev, vel_z, min_vel_z);
  avg_vel_z = (max_vel_z+min_vel_z)/2;

  //Force:
  //X axis:
  max_force_x = trackMax(time_prev, force_x, max_force_x);
  min_force_x = trackMin(time_prev, force_x, min_force_x);
  avg_force_x = (max_force_x+min_force_x)/2;
  //Y axis:
  max_force_y = trackMax(time_prev, force_y, max_force_y);
  min_force_y = trackMin(time_prev, force_y, min_force_y);
  avg_force_y = (max_force_y+min_force_y)/2;
  //Z axis:
  max_force_z = trackMax(time_prev, force_z, max_force_z);
  min_force_z = trackMin(time_prev, force_z, min_force_z);
  avg_force_z = (max_force_z+min_force_z)/2;

  // GENERATE TRANSMISSION STRING
  String data_to_send = "";
  
  //Last ditch effort to transmit the summary after 3 minutes of power - failed to be recieved by waiting laptops even with two recievers
  if(time_prev>180000) {
     // Provide summary 
        data_to_send = data_to_send + "Summary of results" + "\n";
        data_to_send = data_to_send + "Maximum altitude: " + max_alt + "\n";
        data_to_send = data_to_send + "Change in altitude: " + (max_alt-min_alt) + "\n";
                
        //Times
        data_to_send = data_to_send + "Started Pre-Launch at: " + (mode_time[0]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Started Ascending at: " + (mode_time[1]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Started Descending at: " + (mode_time[2]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Time at Landing: " + (mode_time[3]/1000) + "s" + "\n";
        
        //Altitude, Pressure & Temperature
        data_to_send = data_to_send + "Altitude| " + "Max: " + max_alt + ", " + "Min: " + min_alt + ", "  + "Avg: " + avg_alt + "\n";
        data_to_send = data_to_send + "Pressure| " + "Max: " + max_pressure + ", " + "Min: " + min_pressure + ", "  + "Avg: " + avg_pressure + "\n";
        data_to_send = data_to_send + "Temperature| " + "Max: " + max_temp + ", " + "Min: " + min_temp + ", "  + "Avg: " + avg_temp + "\n";

        //Acceleration
        data_to_send = data_to_send + "X Acceleration| " + "Max: " + max_accel_x + ", " + "Min: " + min_accel_x + ", "  + "Avg: " + avg_accel_x + "\n";
        data_to_send = data_to_send + "Y Acceleration| " + "Max: " + max_accel_y + ", " + "Min: " + min_accel_y + ", "  + "Avg: " + avg_accel_y + "\n";
        data_to_send = data_to_send + "Z Acceleration| " + "Max: " + max_accel_z + ", " + "Min: " + min_accel_z + ", "  + "Avg: " + avg_accel_z + "\n";

        //Velocity
        data_to_send = data_to_send + "X Velocity| " + "Max: " + max_vel_x + ", " + "Min: " + min_vel_x + ", "  + "Avg: " + avg_vel_x + "\n";
        data_to_send = data_to_send + "Y Velocity| " + "Max: " + max_vel_y + ", " + "Min: " + min_vel_y + ", "  + "Avg: " + avg_vel_y + "\n";
        data_to_send = data_to_send + "X Velocity| " + "Max: " + max_vel_z + ", " + "Min: " + min_vel_z + ", "  + "Avg: " + avg_vel_z + "\n";

        //Force
        data_to_send = data_to_send + "X Force| " + "Max: " + max_force_x + ", " + "Min: " + min_force_x + ", "  + "Avg: " + avg_force_x + "\n";
        data_to_send = data_to_send + "Y Force| " + "Max: " + max_force_y + ", " + "Min: " + min_force_y + ", "  + "Avg: " + avg_force_y + "\n";
        data_to_send = data_to_send + "Z Force| " + "Max: " + max_force_z + ", " + "Min: " + min_force_z + ", "  + "Avg: " + avg_force_z + "\n";
    }

  }
  
  //Transmission of data
  transmit(data_to_send);
  
  //Mode transmission was commmented out entirely before final launch
  /*
   switch(mode_code){
    case 0: //PRE_LAUNCH
      // Only transmit once every 5s
      if ((time_prev - pre_launch_last_transmit_time) > 5000){
        data_to_send = data_to_send + "Current Mode: PRE_LAUNCH ";
        data_to_send = data_to_send + "," + curr_alt + "," + accel_y;
        data_to_send = data_to_send + "5 seconds since last PRE_LAUNCH transmit - transmit since start: " + time_prev;
       
        // Transmit data
        transmit(data_to_send);

        // Record PRE_LAUNCH transmission time
        pre_launch_last_transmit_time = time_prev;
      }
      else{
        Serial.println(F("5 seconds have not passed since previous PRE_LAUNCH transmit. Wait."));
      }
      break;
      
    case 1: //ASCENDING
      // Transmit as fast as possible
      // Package data into a string
        data_to_send = data_to_send + X_AVERAGED + "," + Y_AVERAGED + "," + Z_AVERAGED + "," + TEMP_AVERAGED + "," + PRESSURE_AVERAGED + "," + curr_alt;
        Serial.println(data_to_send);
      // FORMAT: ACCEL (X,Y,Z), TEMPERATURE, PRESSURE, ALTITUDE
      
      // Transmit data
      transmit(data_to_send);
      break;
      
    case 2: //Descending
      // Transmit as fast as possible
      // Package data into a string
        data_to_send = data_to_send + X_AVERAGED + "," + Y_AVERAGED + "," + Z_AVERAGED + "," + TEMP_AVERAGED + "," + PRESSURE_AVERAGED + "," + curr_alt;
        Serial.println(data_to_send);
      // FORMAT: ACCEL (X,Y,Z), TEMPERATURE, PRESSURE, ALTITUDE
      
      // Transmit data
      transmit(data_to_send);
      break;

    case 3: //LANDED 
    //Transmits the summary only once - as advised on canvas. Possibly would have been better to send the summary every 10 seconds or so
      bool summary=false;
      if (summary==false) {
        // Provide summary 
        data_to_send = data_to_send + "Summary of results" + "\n";
        data_to_send = data_to_send + "Maximum altitude: " + max_alt + "\n";
        data_to_send = data_to_send + "Change in altitude: " + (max_alt-min_alt) + "\n";
                
        //Times
        data_to_send = data_to_send + "Started Pre-Launch at: " + (mode_time[0]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Started Ascending at: " + (mode_time[1]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Started Descending at: " + (mode_time[2]/1000) + "s" + "\n";
        data_to_send = data_to_send + "Time at Landing: " + (mode_time[3]/1000) + "s" + "\n";
        
        //Altitude, Pressure & Temperature
        data_to_send = data_to_send + "Altitude| " + "Max: " + max_alt + ", " + "Min: " + min_alt + ", "  + "Avg: " + avg_alt + "\n";
        data_to_send = data_to_send + "Pressure| " + "Max: " + max_pressure + ", " + "Min: " + min_pressure + ", "  + "Avg: " + avg_pressure + "\n";
        data_to_send = data_to_send + "Temperature| " + "Max: " + max_temp + ", " + "Min: " + min_temp + ", "  + "Avg: " + avg_temp + "\n";

        //Acceleration
        data_to_send = data_to_send + "X Acceleration| " + "Max: " + max_accel_x + ", " + "Min: " + min_accel_x + ", "  + "Avg: " + avg_accel_x + "\n";
        data_to_send = data_to_send + "Y Acceleration| " + "Max: " + max_accel_y + ", " + "Min: " + min_accel_y + ", "  + "Avg: " + avg_accel_y + "\n";
        data_to_send = data_to_send + "Z Acceleration| " + "Max: " + max_accel_z + ", " + "Min: " + min_accel_z + ", "  + "Avg: " + avg_accel_z + "\n";

        //Velocity
        data_to_send = data_to_send + "X Velocity| " + "Max: " + max_vel_x + ", " + "Min: " + min_vel_x + ", "  + "Avg: " + avg_vel_x + "\n";
        data_to_send = data_to_send + "Y Velocity| " + "Max: " + max_vel_y + ", " + "Min: " + min_vel_y + ", "  + "Avg: " + avg_vel_y + "\n";
        data_to_send = data_to_send + "X Velocity| " + "Max: " + max_vel_z + ", " + "Min: " + min_vel_z + ", "  + "Avg: " + avg_vel_z + "\n";

        //Force
        data_to_send = data_to_send + "X Force| " + "Max: " + max_force_x + ", " + "Min: " + min_force_x + ", "  + "Avg: " + avg_force_x + "\n";
        data_to_send = data_to_send + "Y Force| " + "Max: " + max_force_y + ", " + "Min: " + min_force_y + ", "  + "Avg: " + avg_force_y + "\n";
        data_to_send = data_to_send + "Z Force| " + "Max: " + max_force_z + ", " + "Min: " + min_force_z + ", "  + "Avg: " + avg_force_z + "\n";
        transmit(data_to_send);

        summary = true;
      }
      break;

    default:
      break;
      
  }
  */
}