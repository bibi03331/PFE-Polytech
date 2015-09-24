

#include "lib_MPU6050.h"
#include "lib_madgwick.h"
#include "lib_i2c.h"
#include <Servo.h>
#include <Wire.h>

#define Kp_X 40
#define Ki_X 5
#define Kd_X 10

#define Kp_Y 40
#define Ki_Y 5
#define Kd_Y 10

#define Kp_Z 50
#define Ki_Z 10
#define Kd_Z 10

#define offsetX 95
#define offsetY 90
#define offsetZ 100

#define RAD_TO_DEG 180.0f/PI
#define DEG_TO_RAD PI/180.0

// PID
int consigneX, erreurX, somme_erreursX, variation_erreurX, erreur_precedenteX;
int consigneY, erreurY, somme_erreursY, variation_erreurY, erreur_precedenteY;
int consigneZ, erreurZ, somme_erreursZ, variation_erreurZ, erreur_precedenteZ;

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
float aRes, gRes; // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0};   // Bias corrections for gyro
float accelBias[3] = {0, 0, 0}; // Bias corrections for accelerometer
float SelfTest[6];

// parameters for 6 DoF sensor fusion calculations
float pitch, yaw, roll;
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
float deltat = 0.0f;                              // integration interval for both filter schemes

// Create servos instances
Servo servo_X;
Servo servo_Y;
Servo servo_Z;

int initialisation = 1;

void setup()
{
    Wire.begin();
    Serial.begin(38400);

    // Initialisation
    somme_erreursX = (int(offsetX) << 7) / Ki_X;
    somme_erreursY = (int(offsetY) << 7) / Ki_Y;
    somme_erreursZ = (int(offsetZ) << 7) / Ki_Z;

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

void loop()
{
    // If data ready bit set, all data registers have new data
    if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        aRes = getAres(Ascale);

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes;  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes;
        az = (float)accelCount[2] * aRes;

        readGyroData(gyroCount);  // Read the x/y/z adc values
        gRes = getGres(Gscale);

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

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
