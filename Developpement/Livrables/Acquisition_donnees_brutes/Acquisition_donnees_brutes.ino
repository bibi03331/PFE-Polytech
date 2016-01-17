#include <Wire.h>

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L     0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRL     0x14
#define XG_OFFS_USRH     0x15
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define TEMP_OUT_H       0x41
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define FIFO_COUNTH      0x72
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define MPU9250_CAM_ADDRESS  0x68
#define MPU9250_REF_ADDRESS  0x69
#define AK8963_ADDRESS   0x0C

#define DEG_TO_RAD PI/180.0f
#define RAD_TO_DEG 180.0f/PI

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

float accCam[3];
float gyroCam[3];
float magCam[3];

#define ADDR_MAG_CAM_X_MIN  1
#define ADDR_MAG_CAM_X_MAX  5
#define ADDR_MAG_CAM_X_BIAS 9
#define ADDR_MAG_CAM_Y_MIN  13
#define ADDR_MAG_CAM_Y_MAX  17
#define ADDR_MAG_CAM_Y_BIAS 21
#define ADDR_MAG_CAM_Z_MIN  25
#define ADDR_MAG_CAM_Z_MAX  29
#define ADDR_MAG_CAM_Z_BIAS 33
#define ADDR_MAG_REF_X_MIN  37
#define ADDR_MAG_REF_X_MAX  41
#define ADDR_MAG_REF_X_BIAS 45
#define ADDR_MAG_REF_Y_MIN  49
#define ADDR_MAG_REF_Y_MAX  53
#define ADDR_MAG_REF_Y_BIAS 57
#define ADDR_MAG_REF_Z_MIN  61
#define ADDR_MAG_REF_Z_MAX  65
#define ADDR_MAG_REF_Z_BIAS 69

float magCamSensitivity[3] = {0, 0, 0};
float magCamMax[3] = {190.0f, 331.0f, 45.0f};
float magCamMin[3] = {-205.0f, -41.0f, -347.0f};
float gyroCamBias[3] = {-0.3f, 1.0f, -1.0f};
float accCamBias[3] = {0.024f, 0.030f, -0.024f};
float magCamBias[3] = {-7.0f, 145.0f, -151.0f};
float magCamScale[3];

void setup()
{
  Wire.begin();
  //  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_CAM_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

  delay(1000);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    initMPU9250(MPU9250_CAM_ADDRESS);
    writeByte(MPU9250_CAM_ADDRESS, INT_PIN_CFG, 0x02); // Enable Bypass for gyro access
    initAK8963(magCamSensitivity);
    writeByte(MPU9250_CAM_ADDRESS, INT_PIN_CFG, 0x00); // Disable Bypass for gyro access
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }

  initGyroScale(magCamMax, magCamMin, magCamScale);
  
  getAres();
  getGres();
  getMres();
  
}

void loop()
{

  if (readByte(MPU9250_CAM_ADDRESS, INT_STATUS) & 0x01) {

    // Orientation de la camera
    getOrientation(MPU9250_CAM_ADDRESS, accCam, gyroCam, magCam, accCamBias, gyroCamBias, magCamBias, magCamSensitivity, magCamScale);
   
  }

}

void getOrientation(byte address, float * acc, float * gyro, float * mag, float * accBias, float * gyroBias, float * magBias, float * magSensitivity, float * magScale) {

  readAccelData(accelCount, address);  // Read the x/y/z adc values
  // Calculate the accleration value into actual g's
  acc[0] = (float)accelCount[0] * aRes - accBias[0];
  acc[1] = (float)accelCount[1] * aRes - accBias[1];
  acc[2] = (float)accelCount[2] * aRes - accBias[2];

  readGyroData(gyroCount, address);  // Read the x/y/z adc values
  // Calculate the gyro value into actual degrees per second
  gyro[0] = (float)gyroCount[0] * gRes - gyroBias[0];
  gyro[1] = (float)gyroCount[1] * gRes - gyroBias[1];
  gyro[2] = (float)gyroCount[2] * gRes - gyroBias[2];

  writeByte(address, INT_PIN_CFG, 0x02); // Enable Bypass for gyro access
  
  readMagData(magCount);  // Read the x/y/z adc values
  // Calculate the magnetometer values in milliGauss
  mag[0] = ( (float)magCount[0] * mRes * magSensitivity[0] - magBias[0] * mRes * magSensitivity[0] ) * magScale[0];
  mag[1] = ( (float)magCount[1] * mRes * magSensitivity[1] - magBias[1] * mRes * magSensitivity[1] ) * magScale[1];
  mag[2] = ( (float)magCount[2] * mRes * magSensitivity[2] - magBias[2] * mRes * magSensitivity[2] ) * magScale[2];

  writeByte(address, INT_PIN_CFG, 0x00); // Disable Bypass for gyro access
  
  #if(1)
  // Print acceleration values in milligs
  Serial.print("X-acceleration: "); Serial.print(1000 * acc[0]); Serial.print(" mg\t");
  Serial.print("Y-acceleration: "); Serial.print(1000 * acc[1]); Serial.print(" mg\t");
  Serial.print("Z-acceleration: "); Serial.print(1000 * acc[2]); Serial.print(" mg - ");
  
  // Print gyro values in degree/sec
  Serial.print("X-gyro rate: "); Serial.print(gyro[0], 3); Serial.print(" deg/s\t");
  Serial.print("Y-gyro rate: "); Serial.print(gyro[1], 3); Serial.print(" deg/s\t");
  Serial.print("Z-gyro rate: "); Serial.print(gyro[2], 3); Serial.print(" deg/s - ");

  // Print mag values in degree/sec
  Serial.print("X-mag field: "); Serial.print(mag[0]); Serial.print(" mG\t");
  Serial.print("Y-mag field: "); Serial.print(mag[1]); Serial.print(" mG\t");
  Serial.print("Z-mag field: "); Serial.print(mag[2]); Serial.println(" mG");
  
  #endif
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}

void readAccelData(int16_t * destination, byte address)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination, byte address)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(address, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

void initAK8963(float * destination /* magCalibration */)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void initGyroScale(float * magMax, float * magMin, float * magScale) {

  float rawMagScale[3];
  float avgMagScale;

  rawMagScale[0] = (magMax[0] - magMin[0]) / 2;
  rawMagScale[1] = (magMax[1] - magMin[1]) / 2;
  rawMagScale[2] = (magMax[2] - magMin[2]) / 2;
  avgMagScale = (rawMagScale[0] + rawMagScale[1] + rawMagScale[2]) / 3;
  
  magScale[0] = avgMagScale / rawMagScale[0];
  magScale[1] = avgMagScale / rawMagScale[1];
  magScale[2] = avgMagScale / rawMagScale[2];
  
}

void initMPU9250(byte address)
{
  // Reset accel offset 
  writeByte(address, XA_OFFSET_H, 0x00);
  writeByte(address, XA_OFFSET_L, 0x00);
  writeByte(address, YA_OFFSET_H, 0x00);
  writeByte(address, YA_OFFSET_L, 0x00);
  writeByte(address, ZA_OFFSET_H, 0x00);
  writeByte(address, ZA_OFFSET_L, 0x00);

  // Reset accel offset
  writeByte(address, XG_OFFS_USRL, 0x00);
  writeByte(address, XG_OFFS_USRH, 0x00);
  writeByte(address, YG_OFFS_USRL, 0x00);
  writeByte(address, YG_OFFS_USRH, 0x00);
  writeByte(address, ZG_OFFS_USRL, 0x00);
  writeByte(address, ZG_OFFS_USRH, 0x00);

  // Reset device, clear sleep mode, use internal oscillator
  writeByte(address, PWR_MGMT_1, 0x80);
  // Enable all sensors
  writeByte(address, PWR_MGMT_2, 0x00);

  delay(200);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(address, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate

  // Set gyroscope full scale range
  writeByte(address, GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale range configuration
  writeByte(address, ACCEL_CONFIG, 0x00);
  
  writeByte(address, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

