#include <Wire.h> //I2C Arduino Library
#include <SimpleKalmanFilter.h>
#include "PPMEncoder.h"

// time to show values on serial out
unsigned long testpreviousMillis = 0;
unsigned long testcurrentMillis = millis();
const long testinterval = 1000;
int cccount = 0;


const float RAD2DEG = 180.0f / PI;

//  SimpleKalmanFilter(e_mea, e_est, q);
//  e_mea: Measurement Uncertainty 
//  e_est: Estimation Uncertainty 
//  q: Process Noise
SimpleKalmanFilter azimuthKalmanFilter(0.2, 0.2, 0.01);
SimpleKalmanFilter nickKalmanFilter(0.2, 0.2, 0.01);
float Kalman_azi, Kalman_nick;

float filtered_azimuth = 0;
float filtered_nick = 0;

// Center View
int ViewCenterLocked = 0;
int ViewCenterCounter = 0;
float ViewCenter_azimuth = 0;
float ViewCenter_nick = 0;
float temp_ViewCenter_azimuth = 0;
float temp_ViewCenter_nick = 0;
float diff_Center_azimuth = 0;
float diff_Center_nick = 0;

// Output and generation of PPM Signal
int ppm_pos_azimuth = 1500;
int ppm_pos_nick = 1500;
#define OUTPUT_PIN_PPM 10
int ppm_upperEnd_azimuth = 2500;
int ppm_lowerEnd_azimuth = 500;
int ppm_upperEnd_nick = 2500;
int ppm_lowerEnd_nick = 500;


// Setup MPU6050
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t GyX,GyY,GyZ,AcX,AcY,AcZ,Tmp;

int compassx,compassy,compassz; //triple axis data

double gier, nick, azimuth;
double X_h, Y_h;

//I2C address of the QMC5883L
#define ADDR  0x0d
//values for the QMC5883 control register 1
//operating mode
#define Mode_StandBy    0b00000000
#define Mode_Continuous 0b00000001
//Output data rate
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100
//Measure range
#define RNG_2G          0b00000000
#define RNG_8G          0b00010000
//Over sampling rate
#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

void compassSetup()
{
  Wire.begin();
  Serial.println("Start Compass");
  softReset();
  setCtrlRegister(OSR_256,RNG_2G,ODR_200Hz,Mode_Continuous);
  Serial.println("Compass init done");
  return;
}

//function to write data into a register on QMC5883L
void writeRegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(ADDR); //start talking
  Wire.write(reg); 
  Wire.write(val);
  Wire.endTransmission();
}

//function to read results from QMC5883L
void readCompassData(uint16_t * compassx, uint16_t * compassy, uint16_t * compassz) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 6);
  *compassx = Wire.read(); //LSB  x
  *compassx |= Wire.read() << 8; //MSB  x
  *compassy = Wire.read(); //LSB  z
  *compassy |= Wire.read() << 8; //MSB z
  *compassz = Wire.read(); //LSB y
  *compassz |= Wire.read() << 8; //MSB y  
}

//function to set the control register 1 on QMC5883L
void setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode) {
  writeRegister(9,overSampling | range | dataRate | mode);
}

//function to reset QMC5883L
void softReset() {
  writeRegister(0x0a,0x80);
  writeRegister(0x0b,0x01);
}

// 
// Data for MPU6050
//

void gyroSetup()
{ 
  Serial.println("Initializing Gyroscope...");
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Initialization Complete");
  // GetCalibrationData();
  return;
}

//function to read results from Gyroscope
void readGyroData(int16_t * AcX, int16_t * AcY, int16_t * AcZ, int16_t * Tmp, int16_t * GyX, int16_t * GyY, int16_t * GyZ) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  *AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  *AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  *AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  *Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  *GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  *GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  *GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}



//prepare hardware
void setup(){
  Serial.begin(115200);
  compassSetup();
  gyroSetup();
  ppmEncoder.begin(OUTPUT_PIN_PPM);
}

void loop(){
  cccount = cccount + 1;
  
  // get Data
  readCompassData(&compassx, &compassy, &compassz); //read data from sensor
  readGyroData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);

  gier = atan2((double)-AcY, sqrt((long)AcZ*(long)AcZ + (long)AcX*(long)AcX)); // Rechnung Angepasst!!! X und Y Achse des Gyros+ Boards stehen verdreht zueinander
	nick = atan2((double)AcX, sqrt((long)AcZ*(long)AcZ  + (long)AcY*(long)AcY));

  // Calculate Azimuth:
  // Magnetic horizontal components, after compensating for nick(r) and Pitch(p) are:
  // X_h = X*cos(p) + Y*sin(r)*sin(p) + Z*cos(r)*sin(p)
  // Y_h = Y*cos(r) - Z*sin(r)
  // Azimuth = arcTan(Y_h/X_h)

  X_h = (double)compassx*cos(gier) + (double)compassy*sin(nick)*sin(gier) + (double)compassz*cos(nick)*sin(gier);
  Y_h = (double)compassy*cos(nick) - (double)compassz*sin(nick);
  azimuth = atan2(Y_h, X_h);
  if(azimuth < 0) {	/* Convert Azimuth in the range (0, 2pi) */
    azimuth = 2*M_PI + azimuth;
  }
  Kalman_azi = azimuthKalmanFilter.updateEstimate(azimuth);
  Kalman_nick = nickKalmanFilter.updateEstimate(nick);

  
  filtered_azimuth = Kalman_azi;
  filtered_nick = Kalman_nick;
  // Umrechnung zu Grad
  filtered_azimuth = filtered_azimuth * RAD2DEG;
  filtered_nick = filtered_nick * RAD2DEG;


  // if Button or Command - enable Center 
  if(ViewCenterLocked == 0)
  {
    ViewCenterCounter = ViewCenterCounter + 1;
    temp_ViewCenter_azimuth = temp_ViewCenter_azimuth + filtered_azimuth;
    temp_ViewCenter_nick = temp_ViewCenter_nick + filtered_nick;
    ppmEncoder.setChannelPercent(6, 50);
    ppmEncoder.setChannelPercent(7, 50);

    if(ViewCenterCounter == 100)
    {
      ViewCenter_azimuth = temp_ViewCenter_azimuth / 100;
      ViewCenter_nick = temp_ViewCenter_nick / 100;
      ViewCenterCounter = 0;
      ViewCenterLocked = 1;
      temp_ViewCenter_azimuth = 0;
      temp_ViewCenter_nick = 0;
    }
  }else
  {
    diff_Center_azimuth = filtered_azimuth - ViewCenter_azimuth;
    diff_Center_nick = filtered_nick - ViewCenter_nick;
    if(diff_Center_azimuth > 90){diff_Center_azimuth = 90;} 
    if(diff_Center_azimuth < -90){diff_Center_azimuth = -90;} 
    if(diff_Center_nick > 90){diff_Center_nick = 90;} 
    if(diff_Center_nick < -90){diff_Center_nick = -90;} 
    ppm_pos_azimuth = map(diff_Center_azimuth, -90, 90, ppm_lowerEnd_azimuth, ppm_upperEnd_azimuth);
    ppm_pos_nick = map(diff_Center_nick, -90, 90, ppm_lowerEnd_nick, ppm_upperEnd_nick);
    //if(ppm_pos_azimuth > ppm_upperEnd_azimuth){ppm_pos_azimuth = ppm_upperEnd_azimuth;}
    //if(ppm_pos_azimuth < ppm_lowerEnd_azimuth){ppm_pos_azimuth = ppm_lowerEnd_azimuth;}
    //if(ppm_pos_nick > ppm_upperEnd_nick){ppm_pos_nick = ppm_upperEnd_nick;}
    //if(ppm_pos_nick < ppm_lowerEnd_nick){ppm_pos_nick = ppm_lowerEnd_nick;}
    ppmEncoder.setChannel(6, ppm_pos_azimuth);
    ppmEncoder.setChannel(7, ppm_pos_nick);

  }

  testcurrentMillis = millis();
  if (testcurrentMillis - testpreviousMillis >= testinterval)
  {   
    // Show values on serial line
    Serial.println("#### funny Values:");
    //Serial.print("Azimuth: ");
    //Serial.println(azimuth,4);
    //Serial.print("compensated Azimuth: ");
    //Serial.println(Kalman_azi,4);
    //Serial.print("filtered azi: ");
    //Serial.println(filtered_azimuth,4);
    //Serial.print("diff_Center_azimuth: ");
    //Serial.println(diff_Center_azimuth,4);
    //Serial.print("ViewCenter_azimuth: ");
    //Serial.println(ViewCenter_azimuth,4);
    //Serial.print("nick: ");
    //Serial.println(nick,4);
    //Serial.print("compensated nick: ");
    //Serial.println(Kalman_nick,4);
    //Serial.print("filtered nick: ");
    //Serial.println(filtered_nick,4);

    Serial.print("ppm azi: ");
    Serial.println(ppm_pos_azimuth);
    Serial.print("ppm nick: ");
    Serial.println(ppm_pos_nick);

    Serial.print("calculations per second: ");
    Serial.println(cccount);
    Serial.println();
    // Serial.println();
    cccount = 0;
    testpreviousMillis = testcurrentMillis;
  }

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if(inByte == 's')
    {
      ViewCenterLocked = 0;
    }
  }

}
