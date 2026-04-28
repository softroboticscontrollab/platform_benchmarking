/**
 * four_chamber_smoothfeeback.ino
 * (C) Soft Robotics Control Lab 2025
 * Feedback control on four pneumatic chambers using one pump/p-valve pair.
*/

#include <Wire.h> 
#include <Adafruit_I2CDevice.h> 
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MPRLS.h> 
#include <Adafruit_MCP4728.h> // Library: Adafruit_MCP4728

#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin() 
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin 

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN); 
Adafruit_MCP4728 mcp;

// For converting floats to cc register values. If 8-bit PWM...
#define MAX_CC 255
// For 12-bit DAC, this is max, 2^12-1. Note that stored in a uint16 since arduino C doesn't have a uint12.
#define MAX_DAC 4095
// better here than as an int
#define SERIAL_RATE 115200
// for strtok string conversoin
#define BUFSIZE 200

char receivedChars[BUFSIZE];
char tempChars[BUFSIZE]; // temporary array for use when parsing

// Defines how many of each sensor exist via defining the number of limbs
#define n_limbs 2 // number of limbs on robot (1 for test)
#define n_reservoir 1 // number of reservoirs
#define n_sens1 n_reservoir  // number of pressure sensors to reservoir
#define n_sens2 2 * n_limbs // number of pressure sensors to limbs 

#define n_control 2 * n_limbs // one pressure sensor per chamber, one chamber -> one control input

// pin setup and constants

// Only one pump, connected to reservoir
int pump_pin = 5;

// placeholders for what we'll read from the serial terminal
String received;
int raw_duty = 0;
float duty_press = 0;

char *token;
char received_raw[BUFSIZE];

// Time management
unsigned long curr_time;
unsigned long prev_time1;
unsigned long tot_time;
unsigned long max_pwm = 255; //180
unsigned long min_pwm = 0;
unsigned long Es = 0;
// unsigned long dt = 100; 
float dt = 0.1; // seconds
unsigned long sper1 = dt*1000;  // was 50 msec
// storing dac_out in float
float stored_valv[] = {0,0,0,0};

// For holding the sensor data. Modify these arrays for the number of floating point data samples per sensor. Example, combine two temperature sensors into one "sensor."
float dat1[n_sens1]; // reservoir pressure data
float dat2[n_sens2]; // limb pressure data

// P controller data structure for reservoir
float u_rpr[n_sens1];
float err_rpr[n_sens1];
float interr_rpr[n_sens1];
float Prop_rpr[n_sens1];
float Int_rpr[n_sens1];

// Setting pressure goal for reservoir
float goal_rpr[n_sens1] = {1400};

// Continuous controller (valve only) data structures
float valve_apertures[n_control];
// some constants for easy opening/closing of valves
float open_valves[] = {1.0, 1.0, 1.0, 1.0};
float closed_valves[] = {0.0, 0.0, 0.0, 0.0};
// conversions to DAC 12-bit ints
int dac_out = 0;
// other placeholder variables + controller data structures
float valve_e_i = 0.0;
float valve_integral_err[n_control];

// Setting up pin values for on/off valves 
// [fwd0,rev0,fwd1,rev1]
float ABval_pin[] = {46,24,47,25}; // pin 23 has problems
// float u_ABval[n_sens2];

// this array needs to be initialized in setup(). Otherwise, its values are garbage.
// The control code currently runs ALWAYS even before the first string is received over serial, so it's doing
// a computation on garbage values.
// A safe pressure is 0 hPa to start with: all solenoids open, all propvalves... whatever'd.
float defaultpress = 1010.0;
float floatFromPC[n_control]; // bad practice. This is technically correct, but per the variable name, implying that our serial rx is *sensors* but it is really *control inputs

// PID gains for each of the four chambers' proportional valves 
// float Kp[] = {0.18,0.18,0.18,0.18}; // Juan's constants for the pump PID
float kp_rpr = 12.5; // 
float kp_val[] = {0.17, 0.17, 0.17, 0.17};// for example, 100% closed valve (=1) if the difference between desired vs. measured is 100 hPa.used to be 0.008
float ki_rpr = 0.05;
float ki_val[] = {0.005, 0.005, 0.005, 0.005}; // Drew's values: used to be 0.001

// float Kd[] = {0, 0, 0, 0}; // not implemented yet

// Strings that will hold the formatted output of sensor data
String sdata1;
String sdata2;
String sdata3;
// We need to distinguish which set of sensor data we're sending.
String sprefix1 = "RPR";  // for the pressure sensor in reservoir 
String sprefix2 = "MPR"; // for pressure sensor in chambers
String sprefix3 = "VAL"; // for valve openings
//  I2C Mux Variables: 
int comAddr = 0x70;
int comLn = 1;
int comMapMPR[] = {0, 1, 2, 3, 4};

//  Create arrays of sensor objects
Adafruit_MPRLS sens1Arr[n_sens1];
Adafruit_MPRLS sens2Arr[n_sens2];

MCP4728_channel_t channel[] = {MCP4728_CHANNEL_A, MCP4728_CHANNEL_B, MCP4728_CHANNEL_C, MCP4728_CHANNEL_D}; // storing channel locations for the valve drivers in an array

void setup() {
  Serial.begin(SERIAL_RATE);
  Serial.setTimeout(10); // makes read faster
  // Wait until the port is ready (Arduino Leonardo, etc...)
  while (!Serial)
    ;
  delay(1000);
  Serial.println("ezloophw four chamber feedback control. Type <....> to set desired pressures for the number of pumps attached.");
  Serial.println();

  // Initialize the pump outputs. set the pinMode to OUTPUT, and ensure that they begin in the off position.

  pinMode(pump_pin, OUTPUT);
  analogWrite(pump_pin, 0);

 // Setting up pins for on/off (AB) valves
 for(int i = 0; i < n_sens2; i++){ 
    pinMode(ABval_pin[i], OUTPUT);
    digitalWrite(ABval_pin[i],LOW);
  } 

  // Set all initial apertures for the proportional valves to 1.0 (fully open - matches our manual commands below)
  // and also initialize the controller's internval variables
  for(int i=0; i < n_control; i++){
    valve_apertures[i] = 1.0;
    valve_integral_err[i] = 0.0;
  }

  // Low-level controller for reservoir 
  err_rpr[0] = 0.0;
  interr_rpr[0] = 0.0;

  // initialize the "sensor readings" before we get our first string over serial, so controller doesn't go crazy at start.
  for(int i=0; i<n_control; i++){
    floatFromPC[i] = defaultpress;
  }

  // we'll also turn on the LED when any pin is set high
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Time management for sensor sampling:
  prev_time1 = millis();
  curr_time = millis();
  
  // Initialize the sensor data arrays and error data arrays for reservoir
  for(int i=0; i < n_sens1; i++){
    dat1[i] = 0.0;
  }

  // Initialize the sensor data arrays and error data arrays for limb chambers 
  for(int i=0; i < n_sens2; i++){
    dat2[i] = 0.0;
  } 

  Wire.begin();
  int sens1Addr = 0x18;
  int sens2Addr = 0x19;

    comSwitch(comMapMPR[n_sens2+1]);
    // TO DO: better debugging about which sensor. Both address (hardcoded) and switch output (varying.)
    if(!sens1Arr[0].begin()){ //manually set address to begin -- address must be 0x18 for pressure sensors
      Serial.print("ERROR @ reservoir! Unable to initialize pressure sensor at switch output: ");
      Serial.print(n_sens2+1);
      Serial.println(", please check wiring.");
      Serial.println();
      while(true){
      }
    }
    else{
      Serial.print("Pressure sensor for reservoir at address 0x");
      Serial.print(sens1Addr, HEX);
      Serial.println(" initialized successfully");
    }

    for(int i = 0; i < n_sens2; i++){
    comSwitch(comMapMPR[i]);
    // TO DO: better debugging about which sensor. Both address (hardcoded) and switch output (varying.)
    if(!sens2Arr[i].begin()){ //manually set address to begin -- address must be 0x18 for pressure sensors
      Serial.print("ERROR @ limbs! Unable to initialize pressure sensor at switch output: ");
      Serial.print(i);
      Serial.println(", please check wiring.");
      Serial.println();
      while(true){
      }
    }
    else{
      Serial.print("Pressure sensor for chamber at address 0x");
      Serial.print(sens2Addr, HEX);
      Serial.println(" initialized successfully");
    }
    sens2Addr++;
  }

  Serial.println("Testing connection to Adafruit MCP4728!");
// Try to initialize DAC!
  if (!mcp.begin()){
    Serial.println("Failed to find MCP4728 chip, please check wiring");
    while (1){
      delay(10);
     }
   }
  Serial.println("Sucessfully connected to MCP4728 chip");
  
  // Setting desired voltages for each valve controller
  // start fully open to release any excess pressure. Should be max current draw, about 0.5 A with the power supply set to 24V.
  set_valve_apertures(open_valves);
  Serial.println("Setup complete. Waiting 1 sec...");
  delay(1000);

}

void loop() {
  // If there is at least one byte in the serial buffer, someone is trying to send us data, so read that first (don't lose anything!)
  
  curr_time = millis(); // moved from after serial loop
  if( (curr_time - prev_time1) > sper1){
    // Working on the reservoir
    sdata1 = read_sensor1();
    tot_time = prev_time1 + curr_time; 
    prev_time1 = curr_time;
    // Format and send
    datatx(sdata1, sprefix1);

    // Working on the chambers 
    sdata2 = read_sensor2();
    // Format and send
    datatx(sdata2,sprefix2);

    // P controller for pwm to reservoir pump 
    control_rpr();
    // Serial.println(u_rpr[0]);
    // Deciding what to do with pump
    PumpAct();
    
    // Control period is the same as sensing period, since we're doing onboard feedback for pressure signals.
    // Recalculate control signal to proportional valves
    calc_valve_apertures(tot_time);
    // Set the valve apertures (control signals). Send to the DAC.
    set_valve_apertures(valve_apertures);
    // Store values for aperture
    // sdata3 = fmt_dat(stored_valv,n_sens2);
    sdata3 = fmt_dat(valve_apertures,n_sens2);
    // Format and send
    datatx(sdata3, sprefix3);
    
  }
  // then, receive. TO DO this should come first.
  if (recvWithStartEndMarkers()) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
  }
}

int convert_duty(float duty){
  raw_duty = floor(duty * MAX_CC);
  // constrain between 0 and MAX_CC.
  raw_duty = (raw_duty < 0) ? 0 : raw_duty;
  raw_duty = (raw_duty > MAX_CC) ? MAX_CC : raw_duty;
//  Serial.println(raw_duty);
  return raw_duty;
}

String read_sensor1(){
  for(int i = 0; i < n_sens1; i++){
    comSwitch(comMapMPR[n_sens2]);
    if(sens1Arr[i].readStatus()){
      dat1[i] = sens1Arr[i].readPressure();
      }
    }
    return fmt_dat(dat1, n_sens1); 
  }

String read_sensor2(){
  for(int i = 0; i < n_sens2; i++){
    comSwitch(comMapMPR[i]);
    if(sens2Arr[i].readStatus()){
      dat2[i] = sens2Arr[i].readPressure();
    }
  }
  return fmt_dat(dat2, n_sens2); 
}

String fmt_dat(float dat[], int nsens) {
  // format the floats into a space-separated string
  String res = "";
  for(int i=0; i<nsens; i++){
    res = res + String(dat[i]) + " ";
  }
  return res;
}

void datatx(String dat, String prefix) {
  Serial.println("[" + prefix + "]: " + dat);
}

void comSwitch(int j){
  if(j > 7){
    return;
  }
  Wire.beginTransmission(comAddr);
  Wire.write(1 << j);
  Wire.endTransmission();
}

// Calculating our control signal for reservoir pump
void control_rpr(){
  // current pressure error for reservoir
  err_rpr[0] = goal_rpr[0] - dat1[0];
  interr_rpr[0] = interr_rpr[0] + err_rpr[0]*dt;
  // calculare control input
  Prop_rpr[0] = kp_rpr*err_rpr[0];
  // Serial.println(Prop_rpr[0]);
  Int_rpr[0] = ki_rpr*interr_rpr[0];
  // Serial.println(Int_rpr[0]);
  u_rpr[0] = Prop_rpr[0] + Int_rpr[0];
  // limiting values for u_rpr
  if (u_rpr[0] < 0){
    u_rpr[0] = 0;
    }
  if (u_rpr[0] > 255){
    u_rpr[0] = 255;
    }
  }

// Calculating our control signal to the proprotional valves: their aperture percent, [0, 1]
// Returns: none
// State change: valve_apertures[] has new values, recalculated per control law
void calc_valve_apertures(float tot_time){
  for(int i=0; i<n_control; i++){
    // current pressure error for chamber i
    valve_e_i = floatFromPC[i] - dat2[i];
    // add to integral error
    valve_integral_err[i] = valve_integral_err[i] + valve_e_i*dt;
    // u = kp*e + ki*sum(integral_err)
    // note that this is an inverse relationship: larger valve apertures mean dot e<0, so... negative kp!
    valve_apertures[i] = (kp_val[i]*valve_e_i + ki_val[i]*valve_integral_err[i]);
    // Serial.println(valve_apertures[i]);

    if (valve_apertures[i] > 0) {
      digitalWrite(ABval_pin[i],HIGH);
      // Serial.println("Air into chamber");
        }
    else {
        digitalWrite(ABval_pin[i],LOW);
        // Serial.println("Air out of chamber");
        }
    
    if (valve_apertures[i] > 1) {
      valve_apertures[i] = 1;
      }
    if (valve_apertures[i] < -1) {
      valve_apertures[i] = -1;
      }
      
      valve_apertures[i] = abs(valve_apertures[i]);
    }
  }


void PumpAct(){
  analogWrite(pump_pin,u_rpr[0]);
  }

// separate function to set valve apertures, convenient for e.g. manually opening or closing them.
// Input: app[] an array of floats, assumed to have 4 for now. Hardcoded because of the DAC channel naming.
void set_valve_apertures(float app[]){
  mcp.setChannelValue(MCP4728_CHANNEL_A, convert_dac_percent(app[0]));
  //stored_valv[0] = dac_out;
  mcp.setChannelValue(MCP4728_CHANNEL_B, convert_dac_percent(app[1]));
  //stored_valv[1] = dac_out;
  mcp.setChannelValue(MCP4728_CHANNEL_C, convert_dac_percent(app[2]));
  //stored_valv[2] = dac_out;
  mcp.setChannelValue(MCP4728_CHANNEL_D, convert_dac_percent(app[3])); 
  //stored_valv[4] = dac_out; 
}

// State change: receivedChars has new string, rx from serial
bool recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0) {

        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
              if (ndx > BUFSIZE - 2) {
                Serial.print("*******************WARNING**********************");
                Serial.println("");
                Serial.print("Number of characters in input exceed capacity!!!");
                ndx = 0; 
                recvInProgress = false; 
              }
              else{
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= BUFSIZE) {
                    ndx = BUFSIZE - 1;
                }
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                return 1;
            }
        }

        else if (rc == startMarker) {
            // ndx = 0; 
            recvInProgress = true;
        }
    }
    // returns without running if there is no rx data waiting over serial.
    return 0;
}

// State change: floatFromPC now has list of floats corresponding to the received string.
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    static int i = 0;

    for (i = 0; i < n_sens2; i++) {
      if (i == 0){
        strtokIndx = strtok(tempChars, ",");
        floatFromPC[i] = atof(strtokIndx);     // convert this part to a float
      }
      else {
        strtokIndx = strtok(NULL, ",");
        floatFromPC[i] = atof(strtokIndx);     // convert this part to a float
      }
    }
}

void showParsedData() {

    Serial.print("Requested pressure inputs ");
      for (int i = 0; i < n_sens2; i++) {
        Serial.print(floatFromPC[i]);
        Serial.print(" ");
      }
    Serial.println("");

}

// Converts a percentage in [0,1] to a uint16 between [0, 2^12-1] for the DAC, 12 bit.
// This function rectifies negative inputs as well as inputs greater than 1.
int convert_dac_percent(float perc){
  // constrain between 0.0 and 1.0
  perc = (perc < 0) ? 0 : perc;
  perc = (perc > 1.0) ? 1.0 : perc;
  // round to an int
  dac_out = floor(perc * MAX_DAC);
  // // constrain between 0 and MAX_DAC.
  // dac_out = (dac_out < 0) ? 0 : dac_out;
  // dac_out = (dac_out > MAX_DAC) ? MAX_DAC : dac_out;
  // Serial.println(dac_out);
  return dac_out;
}