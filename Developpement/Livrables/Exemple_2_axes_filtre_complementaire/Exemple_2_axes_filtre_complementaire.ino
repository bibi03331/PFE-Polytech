
#include <Servo.h>
#include <Wire.h>

#define Kp_X 1
#define Ki_X 1
#define Kd_X 1

#define Kp_Y 1
#define Ki_Y 1
#define Kd_Y 1

#define Kp_Z 0.4
#define Ki_Z 1
#define Kd_Z 1

#define offsetX 92
#define offsetY 90
#define offsetZ 100

#define compOffX 366
#define compOffY -353
#define compOffZ -728

int cmpt_debug = 0;

// Create servos instances
Servo servo_X;
Servo servo_Y;
Servo servo_Z;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

double gyroXrate, gyroYrate, gyroZrate;
double roll, pitch, yaw;

double compAngleX, compAngleY,compAngleZ;

int consigneX, erreurX, somme_erreursX, variation_erreurX, erreur_precedenteX;
int consigneY, erreurY, somme_erreursY, variation_erreurY, erreur_precedenteY;
int consigneZ, erreurZ, somme_erreursZ, variation_erreurZ, erreur_precedenteZ;

const int MPU6050 = 0x68;
const int HMC5883L = 0x1E;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

float magOffset[3] = { compOffX, compOffY, compOffZ }; // From calibration

void setup() {

  erreurX = 0;
  variation_erreurX = 0;
  erreur_precedenteX = 0;
  erreurY = 0;

  variation_erreurY = 0;
  erreur_precedenteY = 0;

  erreurZ = 0;
  variation_erreurZ = 0;
  erreur_precedenteZ = 0;
  
  compAngleX = 0;
  compAngleY = 0;
  compAngleZ = 0;

  Serial.begin(115200);

  Wire.begin();

  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
  
  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  
  while (i2cRead(MPU6050, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Erreur capteur MPU-6050"));
    while (1);
  }

  while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode

  delay(200); // Wait for sensors to stabilize

  updateMPU6050();
  updateHMC5883L();
  updatePitchRollYaw();

  // Initialisation
  somme_erreursX = (int(offsetX) << 7) / Ki_X;
  somme_erreursY = (int(offsetY) << 7) / Ki_Y;
  somme_erreursZ = (int(offsetZ) << 7) / Ki_Z;

  servo_X.attach(3); // horizontal - roll
  servo_Y.attach(6); // vertical - pitch
  servo_Z.attach(5); // transversal - yaw

  timer = micros();
}

void loop() {
  
  updateMPU6050();
  updateHMC5883L();
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  updatePitchRollYaw();
  
  if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) compAngleX = roll;
  if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) compAngleY = pitch;
  
  /* Estimate angles using complimentary filter */
  compAngleX = 0.96 * (compAngleX + gyroXrate * dt) + 0.04 * roll;
  compAngleY = 0.97 * (compAngleY + gyroYrate * dt) + 0.03 * pitch;
  compAngleZ = 0.9 * (compAngleZ + gyroZrate * dt) + 0.1 * yaw;
  
  computeOrder();

  // updatePO();

  if (cmpt_debug == 50) {
    
    printDebug();
    
    cmpt_debug = 0;
  }
  cmpt_debug++;
  
}

void updatePitchRollYaw() {

  /* Pitch */
  roll  = atan2(accY, accZ) * RAD_TO_DEG; // eq. 25
  
  /* Roll */
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; // eq. 26
  
  /* Yaw */
  yaw = atan2(magY, magX);
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw = ((yaw * RAD_TO_DEG) - 180) * -1;
  
}

void updateMPU6050() {
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = -((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = -(i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = -(i2cData[12] << 8) | i2cData[13];
  
  /* Convert to deg/s */
  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;
  gyroZrate = gyroZ / 131.0;
}

void updateHMC5883L() {
  while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
  magX = ((i2cData[0] << 8) | i2cData[1]);
  magZ = ((i2cData[2] << 8) | i2cData[3]);
  magY = ((i2cData[4] << 8) | i2cData[5]);
  
  magX = magX - magOffset[0];
  magY = magY - magOffset[1];
}

void computeOrder() {

  // Correction PID sur X
  erreurX = compAngleX;
  somme_erreursX += erreurX;
  variation_erreurX = erreurX - erreur_precedenteX;
  consigneX = Kp_X * erreurX + Ki_X * somme_erreursX + Kd_X * variation_erreurX;
  erreur_precedenteX = erreurX;
  consigneX = consigneX >> 7;

  // Correction PID sur Y
  erreurY = compAngleY;
  somme_erreursY += erreurY;
  variation_erreurY = erreurY - erreur_precedenteY;
  consigneY = Kp_Y * erreurY + Ki_Y * somme_erreursY + Kd_Y * variation_erreurY;
  erreur_precedenteY = erreurY;
  consigneY = consigneY >> 7;

  // Correction PID sur Z
  erreurZ = compAngleZ;
  somme_erreursZ += erreurZ;
  variation_erreurZ = erreurZ - erreur_precedenteZ;
  consigneZ = Kp_Z * erreurZ + Ki_Z * somme_erreursZ + Kd_Z * variation_erreurZ;
  erreur_precedenteZ = erreurZ;
  consigneZ = consigneZ >> 7;
}

void updatePO() {

  servo_X.write(consigneX);
  servo_Y.write(consigneY);
  servo_Z.write(consigneZ);
}

void printDebug() {
  
#if 0
    Serial.print( int(consigneX) ); Serial.print(" = Kp_X * "); Serial.print( int(erreurX) ); Serial.print(" + Ki_X * "); Serial.print( int(somme_erreursX) ); Serial.print(" + Kd_X * "); Serial.print( int(variation_erreurX) ); Serial.println();
    Serial.print( int(consigneY) ); Serial.print(" = Kp_Y * "); Serial.print( int(erreurY) ); Serial.print(" + Ki_Y * "); Serial.print( int(somme_erreursY) ); Serial.print(" + Kd_Y * "); Serial.print( int(variation_erreurY) ); Serial.println();
    Serial.print( int(consigneZ) ); Serial.print(" = Kp_Z * "); Serial.print( int(erreurZ) ); Serial.print(" + Ki_Z * "); Serial.print( int(somme_erreursZ) ); Serial.print(" + Kd_Z * "); Serial.print( int(variation_erreurZ) ); Serial.println();
#endif

#if 0
    Serial.print("yaw : "); Serial.print( int(yaw) ); Serial.print(" - ");
    Serial.print("pitch : "); Serial.print( int(pitch) ); Serial.print(" - ");
    Serial.print("roll : "); Serial.print( int(roll) ); Serial.println();
#endif

#if 1
    Serial.print("compAngleX : "); Serial.print( int(compAngleX) ); Serial.print(" ");
    Serial.print("compAngleY : "); Serial.print( int(compAngleY) ); Serial.print(" ");
    Serial.print("compAngleZ : "); Serial.print( int(compAngleZ) ); Serial.println(" ");
#endif

#if 0
    Serial.print("Consigne X : "); Serial.print( int(consigneX) ); Serial.print(" ");
    Serial.print("Consigne Y : "); Serial.print( int(consigneY) ); Serial.print(" ");
    Serial.print("Consigne Z : "); Serial.print( int(consigneZ) ); Serial.print(" "); Serial.println();
#endif
  
}
