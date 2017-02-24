#include <Servo.h>
#include <PID_v1.h>
#include <Math.h>
#include <Wire.h>
#include <SparkFun_ADXL345.h>//accelerometer

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=1.1, Ki=0.1, Kd=0.005;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PID myPID(&Input, &Output, &Setpoint, 0.8, 0.0, 0.3, DIRECT);

const int tilt = 3, pan = 10; // first and second servo

float FIXED_PAN_POS = 95.0, MAX_PAN = 120.0, MIN_PAN = 70.0;//-90deg(70.0)-->0deg(95.0)-->+90deg(120.0)//95.0;//deg:: range 0 (70 anticlockwise), 95 (mid), (140 clockwise) 270
float FIXED_TILT_POS = 140.0, MAX_TILT = 100.0, MIN_TILT = 140.0;//deg:: range 55(up), 140(hor)//145 //accel: -89.5 for 140.0, -48.5 for 100.0

float pan_pos = FIXED_PAN_POS, tilt_pos = FIXED_TILT_POS, new_pan_pos, new_tilt_pos;

float theta_pan = 0, theta_tilt = 0, prev_theta_pan = 0, prev_theta_tilt = 0;

const float D_L2R = 418.0, D_T2B = 317.0;//mm//16.0;//cm //mm//9.0;//cm

long dLEFT, dRIGHT, dLEFTprev, dRIGHTprev, dTOP, dBOTTOM, dTOPprev, dBOTTOMprev, dTOL = 60.0;//mm

bool angle_calc_required = true;

Servo tiltservo, panservo;  // create servo object to control a servo

//Set the pin to receive the signal.
const int pwPinLEFT = 6, pwPinRIGHT = 7, pwPinTOP = 8;

//variables needed to store values
const int arraysize = 51;//31;//31;//51;  //quantity of values to find the median (sample size). Needs to be an odd number
int rangevalue[arraysize], rangevalueLEFT[arraysize], rangevalueRIGHT[arraysize], rangevalueTOP[arraysize], rangevalueBOTTOM[arraysize], rangevalueTILT[arraysize];//int
long pulseLEFT, pulseRIGHT, pulseTOP;

ADXL345 adxl; //variable adxl is an instance of the ADXL345 (accelerometer)library

int x, y, z, rawX, rawY, rawZ;
float X, Y, Z;
float rollrad, pitchrad, rolldeg, pitchdeg;
float pan_accel, tilt_accel;

void setup() {

  // Servo
  tiltservo.attach(tilt);  // attaches the servo
  panservo.attach(pan);  // attaches the servo

  // Inizialize Serial
  Serial.begin(9600);

  //Initial position of the pan and tilt motors:
  panservo.write(pan_pos);
  tiltservo.write(tilt_pos);

  //initialize the variables we're linked to
  Input = tilt_pos;//calc_theta_deg(dLEFT, dRIGHT, D_L2R);

  pinMode(pwPinLEFT, INPUT);
  pinMode(pwPinRIGHT, INPUT);
  pinMode(pwPinTOP, INPUT);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-45.0, 45.0);
  //Turn accelerometer on:
  adxl.powerOn();

  //calculate the initial pan and tilt re-orientation angle:
  rangevalueLEFT[arraysize] = fill_n_sort_distance_array(rangevalueLEFT, pwPinLEFT);
  rangevalueRIGHT[arraysize] = fill_n_sort_distance_array(rangevalueRIGHT, pwPinRIGHT);
  rangevalueTOP[arraysize] = fill_n_sort_distance_array(rangevalueTOP, pwPinTOP);

  dLEFT = mode(rangevalueLEFT, arraysize);
  dRIGHT = mode(rangevalueRIGHT, arraysize) - 3.0; //the sensor on the right is sticking out 3mm with respect to the others
  dTOP = mode(rangevalueTOP, arraysize);
  dBOTTOM = (dLEFT + dRIGHT) / 2;
  
  theta_tilt = calc_theta_deg(dTOP, dBOTTOM, D_T2B);
  Setpoint = map(theta_tilt, 0.0, -41.0, MIN_TILT, MAX_TILT);//new_tilt_pos = map(theta_tilt, 0.0, -41.0, MIN_TILT, MAX_TILT);
  
//  Serial.print("dTOP = ");
//  Serial.println(dTOP);
//  Serial.print("dBOTTOM = ");
//  Serial.println(dBOTTOM);
//  
//  Serial.print("Initial TILT = ");
//  Serial.println(theta_tilt);
//
//  pan_tilt_accel();
//  Serial.print("Accel. TILT = ");
//  Serial.println(pitchdeg);
  
  dTOP = dTOPprev;
  dBOTTOM = dBOTTOMprev;
  prev_theta_tilt = theta_tilt;
  
  delay(3000);
}

void loop() {
  fill_n_sort_distance_array_LRTB(rangevalueLEFT, rangevalueRIGHT, rangevalueTOP, rangevalueBOTTOM, pwPinLEFT, pwPinRIGHT, pwPinTOP);//rangevalueRIGHT[arraysize] = fill_n_sort_distance_array(rangevalueRIGHT, pwPinRIGHT);

  dTOP = mode(rangevalueTOP, arraysize);
  
  //not using pan at the moment, revisit this part of the code when using pan:
  //theta_pan = calc_theta_deg(dLEFT, dRIGHT, D_L2R);
  //new_pan_pos = map(theta_pan, -90.0, 90.0, MIN_PAN, MAX_PAN);

  dBOTTOM = mode(rangevalueBOTTOM, arraysize);//dBOTTOM = (dLEFT + dRIGHT) / 2;--> this is done inside the fill and sort funtion now

  angle_calc_required = sens_flt(dTOPprev, dTOP, dBOTTOMprev, dBOTTOM, dTOL);

  //only calculate a new angle if distances have changed within a certain tolerance:
  if (angle_calc_required){
    theta_tilt = calc_theta_deg(dTOP, dBOTTOM, D_T2B);
    prev_theta_tilt = theta_tilt;
  }
  else{
    theta_tilt = prev_theta_tilt; 
  }

  //reset previous values of distance:
  dTOPprev = dTOP;
  dBOTTOMprev = dBOTTOM;
  
  pan_tilt_accel();
  
  tilt_accel = map(pitchdeg, -48.5, -89.5, 100.0, 140.0);
  Input = tilt_accel;//new_tilt_pos;
  //Setpoint = new_tilt_pos;
  Setpoint = map(theta_tilt, 0.0, -41.0, MIN_TILT, MAX_TILT);
  myPID.Compute();

  new_pan_pos = FIXED_PAN_POS; //reset pan position
  //new_tilt_pos = FIXED_TILT_POS; //reset pan position
  
  panservo.write(new_pan_pos);

  //tiltservo.write(Output + Input); //tiltservo.write(Output);
  
  tiltservo.write(Setpoint);//tiltservo.write(new_tilt_pos);

  //Distances and calculated angle:
//  Serial.print(dTOP/20);
//  Serial.print(",");
//  Serial.print(dBOTTOM/20);
//  Serial.print(",");
//  Serial.println(theta_tilt);

  //Closed loop (with accel.) static
//  Serial.print(tilt_pos);//initial value of tilt
//  Serial.print(",");
//  Serial.print(Setpoint);//where i want to go
//  Serial.print(",");
//  Serial.print(Output + Input);//output for the tilt servo
//  Serial.print(",");
//  Serial.println(tilt_accel);//Input - tilt from the accelerator

  //Closed loop (no accel.) static
//  Serial.print(tilt_pos);//initial value of tilt
//  Serial.print(",");
//  Serial.print(Setpoint);//where i want to go
//  Serial.print(",");
//  Serial.print(Output + Input);//output for the tilt servo
//  Serial.print(",");
//  Serial.println(tilt_accel);//tilt from the accelerator

  //Open loop static
  Serial.print(FIXED_TILT_POS);//initial value of tilt
  Serial.print(",");
  Serial.print(Setpoint);//new_tilt_pos where i want to go
  Serial.print(",");
  Serial.println(Input);//input is the actual tilt from the accel.
  
  //print_all();
}

//int fill_n_sort_distance_array(long pulse, const int pwPin){
int fill_n_sort_distance_array(int *rangevalue, const int pwPin){
  long pulse;
  for (int j = 0; j < arraysize; j++)
  {
    pulse = pulseIn(pwPin, HIGH);
    rangevalue[j] = pulse / 5.8;//for mm /58 for cm
    //delay(1);//originally 10
  }

  isort(rangevalue, arraysize);
  return rangevalue;
}
int fill_n_sort_distance_array_LRTB(int *rangevalueL, int *rangevalueR, int *rangevalueT, int *rangevalueB, const int pwPinL, const int pwPinR, const int pwPinT){
  long pulseL, pulseR, pulseT;
  for (int j = 0; j < arraysize; j++)
  {
    pulseL = pulseIn(pwPinL, HIGH);
    pulseR = pulseIn(pwPinR, HIGH);
    pulseT = pulseIn(pwPinT, HIGH);
    rangevalueL[j] = pulseL / 5.8;//for mm /58 for cm
    rangevalueR[j] = (pulseR / 5.8) - 3.0; //the sensor on the right is sticking out 3mm with respect to the others;//for mm /58 for cm
    rangevalueT[j] = pulseT / 5.8;//for mm /58 for cm
    rangevalueB[j] = (rangevalueL[j] + rangevalueR[j])/2;
    //rangevalueTLT[j] = calc_theta_deg(rangevalueT[j], rangevalueB[j], D_T2B);
    //delay(1);//originally 10
  }

  isort(rangevalueL, arraysize);
  isort(rangevalueR, arraysize);
  isort(rangevalueT, arraysize);
  isort(rangevalueB, arraysize);
  //isort(rangevalueTLT, arraysize);
  
  return rangevalueL, rangevalueR, rangevalueT, rangevalueB;// rangevalueTLT;
}

float calc_theta_deg(long d1, long d2, float D) {
  float theta;
  theta = (atan((d1 - d2) / D)) * (180 / 3.1416); //deg
  return theta;
}

long sens_flt(long d_curr1, long d_prev1, long d_curr2, long d_prev2, long d_tol)
{
  bool a;
  if ((d_curr1 < d_prev1 - d_tol) || (d_curr1 > d_prev1 + d_tol) || (d_curr2 < d_prev2 - d_tol) || (d_curr2 > d_prev2 + d_tol)){
    a = true;
  }
  else{
    a = false;
  }
  return a;
}

//Sorting function
// sort function (Author: Bill Gentles, Nov. 12, 2010)
void isort(int *a, int n) {
  //  *a is an array pointer function

  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

//Mode function, returning the mode or median.
//Author: Jason Lessels
//This work has been compiled using many sources mainly posts/wiki posts from Allen, Bruce (2009/July/23) and Gentles, Bill (2010/Nov/12)
int mode(int *x, int n) {
  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;

  while (i < (n - 1)) {
    prevCount = count;
    count = 0;

    while (x[i] == x[i + 1]) {
      count++;
      i++;
    }

    if (count > prevCount & count > maxCount) {
      mode = x[i];
      maxCount = count;
      bimodal = 0;
    }
    if (count == 0) {
      i++;
    }
    if (count == maxCount) { //If the dataset has 2 or more modes.
      bimodal = 1;
    }
    if (mode == 0 || bimodal == 1) { //Return the median if there is no mode.
      mode = x[(n / 2)];
    }
    return mode;
  }
}

//Accelerometer:
float pan_tilt_accel()
{
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  // Output (x,y,z) on horizontal plane should be approximately (0,0,255)
  // the following 3 lines is for an offset
  rawX = x - 7;
  rawY = y - 6;
  rawZ = z + 10;

  X = rawX / 256.00; // used for angle calculations // PAN
  Y = rawY / 256.00; // used for angle calculations // TILT
  Z = rawZ / 256.00; // used for angle calculations

  rollrad = atan(Y / sqrt(X * X + Z * Z)); // calculated angle in radians
  pitchrad = atan(X / sqrt(Y * Y + Z * Z)); // calculated angle in radians

  rolldeg = 180 * (atan(Y / sqrt(X * X + Z * Z))) / PI; // calculated angle in degrees
  pitchdeg = 180 * (atan(X / sqrt(Y * Y + Z * Z))) / PI; // calculated angle in degrees

  //  Serial.print("\t Angle according to x axis (Roll(deg)) = "); Serial.print(rolldeg);      // calculated angle in degrees
  //  Serial.print("\t Angle according to y axis (Pitch(deg)) = "); Serial.println(pitchdeg);  // calculated angle in degrees

}

//Function to print the arrays.
void printArray(int *a, int n)
{
  for (int i = 0; i < n; i++)
  {
    Serial.print(a[i], DEC);
    Serial.print(' ');
  }

  Serial.println();
}

void print_all() {

    Serial.print("  dLEFT(mm)=");
    Serial.print(dLEFT);
    Serial.print(",");
    Serial.print("  dRIGHT(mm)=");
    Serial.println(dRIGHT);
    Serial.println();
  //  Serial.print("  Theta_PAN(deg)=");
  //  Serial.print(theta_pan);
  //  //Serial.print(",");
  //  Serial.print("  PAN_POS(-)=");
  //  Serial.println(new_pan_pos);
  //  Serial.println();
  //  Serial.print("  PAN_POS_ACCEL(-)=");
  //  Serial.println(rolldeg);
  //  Serial.println();
    Serial.print("  dTOP(mm)=");
    Serial.print(dTOP);
    Serial.print("  dBOTTOM(mm)=");
    Serial.println(dBOTTOM);
    Serial.println();
    Serial.print("  Theta_TILT(deg)=");
    Serial.print(theta_tilt);
    //Serial.print(",");
    Serial.print("  TILT_POS(-)=");
    Serial.println(new_tilt_pos);
//  Serial.print("  TILT_POS_ACCEL(-)=");
//  Serial.println(pitchdeg);
//  Serial.println();
//  Serial.print("  TILT_INPUT(-)=");
//  Serial.println(Input);
//  Serial.println();
  Serial.print("  TILT_OUTPUT(-)=");
  Serial.println(Output+Input);
//  Serial.println();
  //Serial.print(",");
  //Serial.println(Output);
}
