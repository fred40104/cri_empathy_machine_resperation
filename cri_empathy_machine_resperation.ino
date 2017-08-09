// MPU-6050 Accelerometer + Gyro

// Bluetooth module connected to digital pins 2,3
// I2C bus on A4, A5
// Servo on pin 0

#include <Wire.h>
//#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>

#define MPU6050_I2C_ADDRESS 0x68   //first accelerometer
#define MPU6050_I2C_ADDRESS2 0x69  //second accelerometer

#define FREQ  30.0 // sample freq in Hz

// Bluetooth transmitter, used optionally
// SoftwareSerial BTSerial(2, 3); // RX | TX

Servo roll_servo;

// global angle, gyro derived
double gSensitivity = 65.5; // for 500 deg/s, check data sheet
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
int16_t accX = 0, accY = 0, accZ = 0;

double gx2 = 0, gy2 = 0, gz2 = 0;
double gyrX2 = 0, gyrY2 = 0, gyrZ2 = 0;
int16_t accX2 = 0, accY2 = 0, accZ2 = 0;

//initial values
double gxZERO = 0, gyZERO = 0, gzZERO = 0;
double gx2ZERO = 0, gy2ZERO = 0, gz2ZERO = 0;
double zeroCountDown = 100;

//difference between accelerometers
double gxDIF = 0, gyDIF = 0, gzDIF = 0; 

double gyrXoffs = -281.00, gyrYoffs = 18.00, gyrZoffs = -83.00;

void setup()
{      
  int error;
  uint8_t c;
  uint8_t sample_div;

  //BTSerial.begin(38400);
  Serial.begin(115200);

  Serial.println("STARTING");

  // debug led
  pinMode(13, OUTPUT); 

  // servo 
  roll_servo.attach(2, 550, 2550);

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // PWR_MGMT_1:
  // wake up 
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x6b, 0x00);
    i2c_write_reg (MPU6050_I2C_ADDRESS2, 0x6b, 0x00);

  // CONFIG:
  // Low pass filter samples, 1khz sample rate
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x1a, 0x01);
    i2c_write_reg (MPU6050_I2C_ADDRESS2, 0x1a, 0x01);

  // GYRO_CONFIG:
  // 500 deg/s, FS_SEL=1
  // This means 65.5 LSBs/deg/s
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);
    i2c_write_reg(MPU6050_I2C_ADDRESS2, 0x1b, 0x08);

  // CONFIG:
  // set sample rate
  // sample rate FREQ = Gyro sample rate / (sample_div + 1)
  // 1kHz / (div + 1) = FREQ  
  // reg_value = 1khz/FREQ - 1
  sample_div = 1000 / FREQ - 1;
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x19, sample_div);
    i2c_write_reg (MPU6050_I2C_ADDRESS2, 0x19, sample_div);


  Serial.write("Calibrating...");
  digitalWrite(13, HIGH);
  calibrate();
  digitalWrite(13, LOW);
  Serial.write("done.");
}

void loop()
{
  int error;
  double dT;
  double ax, ay, az;
   double ax2, ay2, az2;
  unsigned long start_time, end_time;

  start_time = millis();

  read_sensor_data();

  // angles based on accelerometer
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

  ay2 = atan2(accX2, sqrt( pow(accY2, 2) + pow(accZ2, 2))) * 180 / M_PI;
  ax2 = atan2(accY2, sqrt( pow(accX2, 2) + pow(accZ2, 2))) * 180 / M_PI;

  // angles based on gyro (deg/s)
  gx = gx + gyrX / FREQ;
  gy = gy - gyrY / FREQ;
  gz = gz + gyrZ / FREQ;

  gx2 = gx2 + gyrX2 / FREQ;
  gy2 = gy2 - gyrY2 / FREQ;
  gz2 = gz2 + gyrZ2 / FREQ;

  // complementary filter
  // tau = DT*(A)/(1-A)
  // = 0.48sec
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;

  gx2 = gx2 * 0.96 + ax2 * 0.04;
  gy2 = gy2 * 0.96 + ay2 * 0.04;
  
  //apply init values to zero for position
  /* gx = gx - gxZERO; 
  gy = gy - gyZERO;
  gz = gz - gzZERO;
     
  gx2 = gx2 - gx2ZERO;
  gy2 = gy2 - gy2ZERO;
  gz2 = gz2 - gz2ZERO; */

  //Difference between accelerometers - removes body movement from resperation signal
  gxDIF = abs(gx - gx2) - abs(gxZERO - gx2ZERO); 
  gyDIF = abs(gy - gy2) - abs(gyZERO - gy2ZERO); 
  gzDIF = abs(gz - gz2) - abs(gzZERO - gz2ZERO); 

      digitalWrite(13, HIGH);
      
      Serial.print("DIFFERENCE: ");
      Serial.print(gxDIF, 2);
      Serial.print(", ");
      Serial.print(gyDIF, 2);
      Serial.print(", ");
      Serial.print(gzDIF, 2);

      Serial.print("  1st: ");
      Serial.print(gx, 2);
      Serial.print(", ");
      Serial.print(gy, 2);
      Serial.print(", ");
      Serial.print(gz, 2);

      Serial.print("  2nd: ");
      Serial.print(gx2, 2);
      Serial.print(", ");
      Serial.print(gy2, 2);
      Serial.print(", ");
      Serial.print(gz2, 2);
      
      Serial.print("  ZERO: ");
      Serial.print(gx2ZERO, 2);
      Serial.print(", ");
      Serial.print(gy2ZERO, 2);
      Serial.print(", ");
      Serial.print(gz2ZERO, 2);
      Serial.print("  Time till set zeros: ");
      Serial.println(zeroCountDown, 2);
 
  roll_servo.write(-gxDIF+90);  //Servo controlled by difference between accelerometers
  
  //get initial values to zero out current values so we can see difference
  //we have to wait and count down because it takes a while for the accelerometers to calibrate
  if(gxZERO == 0 && zeroCountDown <= 0){
    
      gxZERO = gx; 
      gyZERO = gy;
      gzZERO = gz;
     
      gx2ZERO = gx2; 
      gy2ZERO = gy2;
      gz2ZERO = gz2;
  }
  if(zeroCountDown > 0){ zeroCountDown--; }

  end_time = millis();

  // remaining time to complete sample time
  delay(((1/FREQ) * 1000) - (end_time - start_time));
  //Serial.println(end_time - start_time);
  
  digitalWrite(13, LOW);
}


void calibrate(){

  int x;
  long xSum = 0, ySum = 0, zSum = 0;
    long xSum2 = 0, ySum2 = 0, zSum2 = 0;
  uint8_t i2cData[6]; 
    uint8_t i2cData2[6]; 
  int num = 500;
  uint8_t error;
    uint8_t error2;

  for (x = 0; x < num; x++){

    error = i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, 6);
    if(error!=0)
    return;

    error2 = i2c_read(MPU6050_I2C_ADDRESS2, 0x43, i2cData2, 6);
    if(error2!=0)
    return;

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);

    xSum2 += ((i2cData2[0] << 8) | i2cData2[1]);
    ySum2 += ((i2cData2[2] << 8) | i2cData2[3]);
    zSum2 += ((i2cData2[4] << 8) | i2cData2[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

//  Serial.println("Calibration result:");
//  Serial.print(gyrXoffs);
//  Serial.print(", ");
//  Serial.print(gyrYoffs);
//  Serial.print(", ");//  Serial.println(gyrZoffs);
  
} 

void read_sensor_data(){
 uint8_t i2cData[14];
  uint8_t i2cData2[14];
  
 uint8_t error;
 // read imu data
 error = i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, 14);
 if(error!=0)
 return;

  error = i2c_read(MPU6050_I2C_ADDRESS2, 0x3b, i2cData2, 14); //2nd Accelerometer
 if(error!=0)
 return;

 // assemble 16 bit sensor data
 accX = ((i2cData[0] << 8) | i2cData[1]);
 accY = ((i2cData[2] << 8) | i2cData[3]);
 accZ = ((i2cData[4] << 8) | i2cData[5]);

 accX2 = ((i2cData2[0] << 8) | i2cData2[1]);
 accY2 = ((i2cData2[2] << 8) | i2cData2[3]);
 accZ2 = ((i2cData2[4] << 8) | i2cData2[5]);

 gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
 gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
 gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;

 gyrX2 = (((i2cData2[8] << 8) | i2cData2[9]) - gyrXoffs) / gSensitivity;
 gyrY2 = (((i2cData2[10] << 8) | i2cData2[11]) - gyrYoffs) / gSensitivity;
 gyrZ2 = (((i2cData2[12] << 8) | i2cData2[13]) - gyrZoffs) / gSensitivity;
 
}

// ---- I2C routines

int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);
  if (n != 1)
  return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
  return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
  return (-11);

  return (0);  // return : no error
}


int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);        // write the start address
  if (n != 1)
  return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
  return (error);

  return (0);         // return : no error
}


int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;
  
  error = i2c_write(addr, reg, &data, 1);
  return (error);
}

