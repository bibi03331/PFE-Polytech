
#include <Wire.h>
#include <Servo.h>

#define Kp_X 40
#define Ki_X 5
#define Kd_X 10

#define Kp_Y 40
#define Ki_Y 5
#define Kd_Y 10

#define Kp_Z 50
#define Ki_Z 10
#define Kd_Z 10

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
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define MPU6050_ADDRESS  0x68

#define offsetX 95
#define offsetY 90
#define offsetZ 100

#define RAD_TO_DEG 180.0f/PI
#define DEG_TO_RAD PI/180.0

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

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
float aRes, gRes; // scale resolutions per LSB for the sensors

// Pin definitions
#define intPin 2

// Data acquisition
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

// PID calculation
int consigneX, erreurX, somme_erreursX, variation_erreurX, erreur_precedenteX;
int consigneY, erreurY, somme_erreursY, variation_erreurY, erreur_precedenteY;
int consigneZ, erreurZ, somme_erreursZ, variation_erreurZ, erreur_precedenteZ;

// Parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float pitch, yaw, roll;
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

// Create servos instances
Servo servo_X;
Servo servo_Y;
Servo servo_Z;

int initialisation = 1;
int dataReady = 0;

void setup()
{
    Wire.begin();
    Serial.begin(38400);
    
    // Initialisation
    somme_erreursX = (int(offsetX) << 7) / Ki_X;
    somme_erreursY = (int(offsetY) << 7) / Ki_Y;
    somme_erreursZ = (int(offsetZ) << 7) / Ki_Z;

    attachInterrupt(digitalPinToInterrupt(intPin), dataFromIMU, CHANGE);
    
    servo_X.attach(3); // horizontal - roll
    servo_Y.attach(6); // vertical - pitch
    servo_Z.attach(5); // transversal - yaw
    
    if (true)
    {
        MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            
            delay(1000);
            calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            delay(1000);
            initMPU6050();
            Serial.println("MPU6050 initialized for active data mode....");
        }
        else
        {
            Serial.print("Could not connect to MPU6050: 0x");
            
            while(1) ; // Loop forever if communication doesn't happen
        }
    }
}

void dataFromIMU() {
  dataReady = 1;
}

void loop()
{
    // If data ready bit set, all data registers have new data
    if ( (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) && dataReady == 1) {  // check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();
        
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes;  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes;
        az = (float)accelCount[2] * aRes;
        
        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();
        
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        dataReady = 0;
    }
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    MadgwickQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD);
    
    yaw   = ( atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) ) * RAD_TO_DEG;
    pitch = ( -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) ) * RAD_TO_DEG;
    roll  = ( atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) ) * RAD_TO_DEG;
    
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    if ( initialisation == 1 && ( (yaw < 1.0 && yaw > -1.0) && (pitch < 1.0 && pitch > -1.0) || (roll < 1.0 && roll > -1.0) ) )  {
      // Fin initialisation
      initialisation = 0;
    }
    
    if ( initialisation == 0 ) {
      computeOrder();
      updatePO();
    }
    
}

void updatePO() {

  servo_X.write(consigneX);
  servo_Y.write(consigneY);
  servo_Z.write(consigneZ);
}

void computeOrder() {

  // Correction PID sur X
  erreurX = -roll;
  somme_erreursX += erreurX;
  variation_erreurX = erreurX - erreur_precedenteX;
  consigneX = Kp_X * erreurX + Ki_X * somme_erreursX + Kd_X * variation_erreurX;
  erreur_precedenteX = erreurX;
  consigneX = consigneX >> 7;

  // Correction PID sur Y
  erreurY = pitch;
  somme_erreursY += erreurY;
  variation_erreurY = erreurY - erreur_precedenteY;
  consigneY = Kp_Y * erreurY + Ki_Y * somme_erreursY + Kd_Y * variation_erreurY;
  erreur_precedenteY = erreurY;
  consigneY = consigneY >> 7;

  // Correction PID sur Z
  erreurZ = yaw;
  somme_erreursZ += erreurZ;
  variation_erreurZ = erreurZ - erreur_precedenteZ;
  consigneZ = Kp_Z * erreurZ + Ki_Z * somme_erreursZ + Kd_Z * variation_erreurZ;
  erreur_precedenteZ = erreurZ;
  consigneZ = consigneZ >> 7;
}

//===================================================================================================================
//====== Set of useful function to access acceleratio, gyroscope, and temperature data
//===================================================================================================================

void getGres() {
    switch (Gscale)
    {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
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
            aRes = 2.0/32768.0;
            break;
        case AFS_4G:
            aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            aRes = 8.0/32768.0;
            break;
        case AFS_16G:
            aRes = 16.0/32768.0;
            break;
    }
}

void readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);     // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
    destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);      // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
    destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

int16_t readTempData()
{
    uint8_t rawData[2];
    readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return ((int16_t)rawData[0]) << 8 | rawData[1] ;         // Turn the MSB and LSB into a 16-bit value
}

void initMPU6050()
{
    // wake up device-don't need this here if using calibration function below
    //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
    
    // get stable time source
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    
    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    writeByte(MPU6050_ADDRESS, CONFIG, 0x03);
    
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
    
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
    
    // Set accelerometer configuration
    c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer
    
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float * dest1, float * dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    
    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
    
    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);
    
    // Configure device for bias calculation
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);
    
    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;     // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;   // = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes
    
    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
    
    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
        
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
    
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
    
    // Push gyro biases to hardware registers
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);
    
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
    
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    
    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
    
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
    
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    
    // Push accelerometer biases to hardware registers
    writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
    writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
    writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);
    
    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];
    
    // Configure the accelerometer for self-test
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);  // Delay a while to let the device execute the self-test
    rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
    rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
    
    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }
    
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
    Wire.write(subAddress);                   // Put slave register address in Tx buffer
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
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error
    
    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    
    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;
    
    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;
    
    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

