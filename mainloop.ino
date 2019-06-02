/*
 * This file is part of the Carnegie Mellon Rocket Command  
 * apogee targeting system (ATS) project, an active airbraking 
 * system used to allow a high power rocket to reach an apogee 
 * of precisely 5100 feet.
 * 
 * Units:
 * feet, slugs, seconds
 * 
 * It contains the code for the ATS initialization and mainloop 
 * running on the onboard Arduino Uno microcontroller. Over the course 
 * of the launch vehicle's flight this program will perform the 
 * following actions upon execution:
 * 
 * 1. Waits for launch to be detected
 * 2. Waits until motor burnout
 * 3. Performs control on apogee and records data
 * 4. Retracts flaps at apogee
 * 
 * ---------------------------------------------------------------------------
 * ---------------------------------------------------------------------------
 * setup:
 * The setup section of this file 
 * 
 * 1. Initializes all onboard sensors and readies them to collect state data. 
 *    These sensors are individually read by selecting their corresponding
 *    pin on the input of an I2C multiplexer chip connected to the Uno.
 * 2. Waits in a loop, exiting when the rocket surpasses 100 feet above 
 *    ground level (AGL).
 * 3. Counts down 3.25 seconds (the amount of time for which the motor burns)
 * ---------------------------------------------------------------------------
 * ---------------------------------------------------------------------------
 * 
 * ---------------------------------------------------------------------------
 * ---------------------------------------------------------------------------
 * loop:
 * The mainloop section of this file
 * 
 * 1. Records state data
 * 2. performs a prediction of apogee based on collected state data
 * 3. Enacts control on apogee based on proportional and integral 
 *    errors on predicted apogee
 * 4. Logs a number of data metrics to an SD card
 * 5. Exits once apogee is detected
 * ---------------------------------------------------------------------------
 * ---------------------------------------------------------------------------
 */


// include struct declarations
#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H
#include <common_definitions.h>
#endif

//include control system libraries
#include <LVD.h>
#include <pi_controller.h>
#include <sensor.h>

//include sensor libraries
#include <Wire.h>
#include <SparkFunMPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//include other libraries
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

//---------------------------------------------------------------------------------------------------
#define TCAADDR          0x70

#define FULL_E           154.69
#define FULL_R           90.00
#define HALF             110.41

/* the number of states we must register at a velocity less than 0
 * to determine that we have reached apogee
 */
#define MIN_N_NEGATIVE_V 10

// last index of all the I2C pins on the I2C multiplexer chip
#define LAST_I2C_PIN     7
//---------------------------------------------------------------------------------------------------

// prediction and control class initialization

double drag_model[N_D_IMPACTORS][N_D_COEFF_TERMS] = 
{ {0.4298,0,0,0,0}, {1.1299,-6.948,76.458,0,0}, {0,0,0,0,0} };

LVD r = LVD(1.4966, 0.125, 0.2570, 3,
            drag_model,
            32., 0.00237,
            5100., 0.104);

double servo_ext_model[N_SERVO_COEFFS] = 
{1.5708, 5.299, -1.757, 900.8, -3481, -4.481e+5, 
 4.261e+6, 6.36e+7, -1.077e+9, 3.967e+9};
 
pi_controller c = pi_controller(1.E-5, 2.E-4, servo_ext_model, 0.125);

// sensor data class initialization
sensor s = sensor();
Adafruit_BNO055 bno = Adafruit_BNO055();
MPL3115A2 mpl1;
MPL3115A2 mpl2;
MPL3115A2 mpl3;
MPL3115A2 mpl4;

Servo control_servo;

//---------------------------------------------------------------------------------------------------

// prediction and control variable initialization

state_t X_tm1 = new state;  // last state
state_t X_t = new state;    // current state
state_t X_tp = new state;   // current predicted state

double U_t = 0;
double e_t = 0;
error_t e = new error;

uint8_t n_negative_v = 0;

// sensor variable initialization

double init_alt_1;
double init_alt_2;
double init_alt_3;
double alt_t1;
double alt_t2;
double alt_t3;
double alt_t4;

double init_theta;
double th_t3_y;
double th_t3_z;

String telemetry0;
String telemetry1;
String telemetry2;
String telemetry3;
String telemetry4;


uint32_t start_time;
uint32_t end_time;
//---------------------------------------------------------------------------------------------------

// start up procedure and launch detection loop
void setup() {
  Wire.begin();

  // initialize altimeters
  tcaselect(2);
  mpl1.begin();
  mpl1.setModeAltimeter(); 
  mpl1.setOversampleRate(7);
  mpl1.enableEventFlags(); 
  
  tcaselect(3);
  mpl2.begin(); 
  mpl2.setModeAltimeter(); 
  mpl2.setOversampleRate(7); 
  mpl2.enableEventFlags(); 
   
  tcaselect(5);
  mpl3.begin(); 
  mpl3.setModeAltimeter(); 
  mpl3.setOversampleRate(7); 
  mpl3.enableEventFlags(); 
 
  tcaselect(6);
  mpl4.begin(); 
  mpl4.setModeAltimeter(); 
  mpl4.setOversampleRate(7); 
  mpl4.enableEventFlags(); 

  // initialize inertial measurement unit
  tcaselect(4);
  bno.begin();
  
  control_servo.attach(5); // initialize servo
  SD.begin(8);             // initialize SD card

  delay(2000);

  // extension range testing
  control_servo.write(FULL_E);
  delay(1000);
  control_servo.write(HALF);
  delay(1000);
  control_servo.write(FULL_R);
  
  // zeroing altitude state AGL
  tcaselect(2);
  alt_t1 = (double)mpl1.readAltitudeFt();
  tcaselect(3);
  alt_t2 = (double)mpl2.readAltitudeFt();
  tcaselect(5);
  alt_t3 = (double)mpl3.readAltitudeFt();
  tcaselect(6);
  alt_t4 = (double)mpl4.readAltitudeFt();
  tcaselect(4);
  th_t3_y = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
  th_t3_z = bno.getVector(Adafruit_BNO055::VECTOR_EULER).z();

  s.zero_altimeters(alt_t1, alt_t2, alt_t3, alt_t4);

  X_tm1->velocity = 0;
  X_tm1->altitude = 0;
  X_tm1->theta = 0;
  
//---------------------------------------------------------------------------------------------------
  // pre-loop flight operation
  // hold until we are 100 feet off of the ground
  while(s.get_altitude(X_tm1, alt_t1,alt_t2,alt_t3,alt_t4) < 100){
    tcaselect(2);
    alt_t1 = (double)mpl1.readAltitudeFt();
    tcaselect(3);
    alt_t2 = (double)mpl2.readAltitudeFt();
    tcaselect(5);
    alt_t3 = (double)mpl3.readAltitudeFt();
    tcaselect(6);
    alt_t4 = (double)mpl4.readAltitudeFt();
  }
  
  // wait for motor burnout to occur and then initialize most recent state
  delay(3250);

  // guess what burnout velocity is
  X_tm1->velocity = 600; 

  tcaselect(2);
  alt_t1 = (double)mpl1.readAltitudeFt();
  tcaselect(3);
  alt_t2 = (double)mpl2.readAltitudeFt();
  tcaselect(5);
  alt_t3 = (double)mpl3.readAltitudeFt();
  tcaselect(6);
  alt_t4 = (double)mpl4.readAltitudeFt();
  init_alt_1 = s.get_altitude(X_tm1, alt_t1, alt_t2, alt_t3, alt_t4);
  delay(SAMPLE_T*1000);

  tcaselect(2);
  alt_t1 = (double)mpl1.readAltitudeFt();
  tcaselect(3);
  alt_t2 = (double)mpl2.readAltitudeFt();
  tcaselect(5);
  alt_t3 = (double)mpl3.readAltitudeFt();
  tcaselect(6);
  alt_t4 = (double)mpl4.readAltitudeFt();
  init_alt_2 = s.get_altitude(X_tm1, alt_t1, alt_t2, alt_t3, alt_t4);

  tcaselect(4);
  th_t3_y = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
  th_t3_z = bno.getVector(Adafruit_BNO055::VECTOR_EULER).z();
  init_theta = s.get_polar_angle(th_t3_y, th_t3_z);

  // finally, we initialize the last state
  
  X_tm1->velocity = (init_alt_2-init_alt_1)/SAMPLE_T; 
  X_tm1->altitude = init_alt_2;
  X_tm1->theta = init_theta;

  delay(SAMPLE_T*1000);

  X_t->velocity = 0;
  X_t->altitude = 0;
  X_t->theta = 0;

  X_tp->velocity = 0;
  X_tp->altitude = 0;
  X_tp->theta = 0;

  e->inst = 0;
  e->acc = 0;   
}

//---------------------------------------------------------------------------------------------------
// enter the main control loop
void loop() {
  /* if we have found several states to have a velocity less than 0,
   *  we determine that apogee has occurred
   */
  if(n_negative_v < MIN_N_NEGATIVE_V){
    start_time = millis();
    
    // read sensors
    tcaselect(2);
    alt_t1 = (double)mpl1.readAltitudeFt();
    tcaselect(3);
    alt_t2 = (double)mpl2.readAltitudeFt();
    tcaselect(5);
    alt_t3 = (double)mpl3.readAltitudeFt();
    tcaselect(6);
    alt_t4 = (double)mpl4.readAltitudeFt();
    tcaselect(4);
    th_t3_y = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
    th_t3_z = bno.getVector(Adafruit_BNO055::VECTOR_EULER).z();
    
    // process raw data to get state data
    X_t->velocity = s.get_vertical_speed(X_tm1,alt_t1,alt_t2,alt_t3,alt_t4);
    X_t->altitude = s.get_altitude(X_tm1, alt_t1,alt_t2,alt_t3,alt_t4);
    X_t->theta = s.get_polar_angle(th_t3_y, th_t3_z);

    // retract flaps if apogee is reached
    if(X_t->velocity < 0){
      n_negative_v+=1;
    }

    // make a prediction of apogee
    r.ms_predict(X_t, X_tp, U_t);

    // calcaulate error 
    e_t = X_tp->altitude-r.get_apo_goal();
    e->inst = e_t;
    e->acc += e_t;

    // calculate control
    U_t = c.get_extension(e);
    // enact control
    control_servo.write(c.get_theta(e));

    // save current state to old state
    X_tm1->velocity = X_t->velocity;
    X_tm1->altitude = X_t->altitude;
    X_tm1->theta = X_t->theta;


    // produce telemetry strings to be recorded to the SD card
    telemetry0 = "t0" + String(X_t->velocity) + " " + String(X_t->altitude) + " " + String(X_t->theta);
    
    telemetry1 = "t1" + String(X_tp->velocity) + " " + String(X_tp->altitude) + " " + String(X_tp->theta);
                
    telemetry2 = "t2" + String(alt_t1,3) + " " + String(alt_t2,3) + " " + String(alt_t3,3) + " " + String(alt_t4,3);

    telemetry3 = "t3" + String(th_t3_y,3) + " " + String(th_t3_z,3) + " " + String(X_t->theta,3);

    telemetry4 = "t4" +String(e->inst) + " " + String(e->acc) + " " + String(U_t);


    File dataFile = SD.open("ATS_data.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(telemetry0);
      dataFile.println(telemetry1);
      dataFile.println(telemetry2);
      dataFile.println(telemetry3);
      dataFile.println(telemetry4);
      dataFile.close();
    }
    
        
    end_time = millis();

    /* delay by the appropriate amount to ensure that the mainloop runs with a
     * constant period of SAMPLE_T seconds
     */
    if((end_time - start_time) <= (SAMPLE_T*1000)){
      delay(SAMPLE_T*1000-end_time+start_time);
    }
  }
}


// a few functions for miscellaneous system controls tasks
//---------------------------------------------------------------------------------------------------

/* 
 * selects address to be used for the current device -
 * this is used to get around the issue that the altimeters
 * all have the same I2C address.
  */
void tcaselect(uint8_t i) {
  if (i > LAST_I2C_PIN) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
