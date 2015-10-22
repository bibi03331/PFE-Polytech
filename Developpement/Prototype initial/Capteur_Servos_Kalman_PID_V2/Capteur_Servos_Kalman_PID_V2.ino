
#include <Servo.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter


#define Kp_X 18
#define Ki_X 9
#define Kd_X 3

#define Kp_Y 12
#define Ki_Y 8
#define Kd_Y 7

#define Kp_Z 0.1
#define Ki_Z 1
#define Kd_Z 4

int cmpt = 0;

// Create the Kalman instances
Kalman kalmanX;
Kalman kalmanY;

// Create servos instances
Servo servo_X;
Servo servo_Y;
Servo servo_Z;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

// double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX = 0; // Calculated angle using a Kalman filter
double kalAngleY = 0; 

int consigneX, erreurX, somme_erreursX, variation_erreurX, erreur_precedenteX;
int consigneY, erreurY, somme_erreursY, variation_erreurY, erreur_precedenteY;
int consigneZ, erreurZ, somme_erreursZ, variation_erreurZ, erreur_precedenteZ;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {

  erreurX = 0;
  somme_erreursX = (11785 / Ki_X);
  variation_erreurX = 0;
  erreur_precedenteX = 0;
  
  erreurY = 0;
  somme_erreursY = (11538 / Ki_Y);
  variation_erreurY = 0;
  erreur_precedenteY = 0;
  
  erreurZ = 0;
  somme_erreursZ = (13000 / Ki_Z);
  variation_erreurZ = 0;
  erreur_precedenteZ = 0;
  
  Serial.begin(115200);

  Wire.begin();

  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(1000); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source : http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  double roll  = atan2(accY, accZ) * RAD_TO_DEG; // eq. 25
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; // eq.26

  // Set starting angle
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  // compAngleX = roll;
  // compAngleY = pitch;

  servo_X.attach(3); // horizontal - roll
  servo_Y.attach(6); // vertical - pitch
  servo_Z.attach(5); // transversal - yaw

  // Mise en position initiale des servo-moteurs
  //servo_X.write(90);
  servo_Y.write(92);
  servo_Z.write(100);

  timer = micros();
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll  = atan2(accY, accZ) * RAD_TO_DEG; // eq. 25
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; // eq. 26

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    // compAngleX = roll;
    kalAngleX = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }

  if ( abs(kalAngleX) > 90 ) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  // Calculate the angle using a Complimentary filter
  // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  // compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Correction PID sur X
  //erreurX = -compAngleX;
  erreurX = -kalAngleX;
  somme_erreursX += erreurX;
  variation_erreurX = erreurX - erreur_precedenteX;
  consigneX = Kp_X * erreurX + Ki_X * somme_erreursX + Kd_X * variation_erreurX;
  erreur_precedenteX = erreurX;
  consigneX = consigneX >> 7;

  // Correction PID sur Y
  erreurY = kalAngleY;
  //erreurY = compAngleY; 
  somme_erreursY += erreurY;
  variation_erreurY = erreurY - erreur_precedenteY;
  consigneY = Kp_Y * erreurY + Ki_Y * somme_erreursY + Kd_Y * variation_erreurY;
  erreur_precedenteY = erreurY;
  consigneY = consigneY >> 7;

  // Correction PID sur Z
  erreurZ = gyroZrate;
  somme_erreursZ += erreurZ;
  variation_erreurZ = erreurZ - erreur_precedenteZ;
  consigneZ = Kp_Z * erreurZ + Ki_Z * somme_erreursZ + Kd_Z * variation_erreurZ;
  erreur_precedenteZ = erreurZ;
  consigneZ = consigneZ >> 7;

  servo_X.write(consigneX);
  servo_Y.write(consigneY);
  servo_Z.write(consigneZ);

  
  // Debug
  if (cmpt == 10) {
    
    Serial.print("gyroZrate : "); Serial.print( int(gyroZrate) ); Serial.println();
    
    /*
    Serial.print( int(consigneX) ); Serial.print(" = Kp_X * "); Serial.print( int(erreurX) ); Serial.print(" + Ki_X * "); Serial.print( int(somme_erreursX) ); Serial.print(" + Kd_X * "); Serial.print( int(variation_erreurX) ); Serial.println();
    Serial.print( int(consigneY) ); Serial.print(" = Kp_Y * "); Serial.print( int(erreurY) ); Serial.print(" + Ki_Y * "); Serial.print( int(somme_erreursY) ); Serial.print(" + Kd_Y * "); Serial.print( int(variation_erreurY) ); Serial.println();
    */
    
    /*
    Serial.print("kalAngleX : "); Serial.print( int(kalAngleX) ); Serial.print(" ");
    Serial.print("kalAngleY : "); Serial.print( int(kalAngleY) ); Serial.print(" ");
    Serial.print("gyroZrate : "); Serial.print( int(gyroZrate) ); Serial.print(" ");
    
    Serial.print("compAngleX : "); Serial.print( int(kalAngleX) ); Serial.print(" ");
    Serial.print("compAngleY : "); Serial.print( int(kalAngleY) ); Serial.print(" ");
    
    Serial.print("Consigne X : "); Serial.print( int(consigneX) ); Serial.print(" ");
    Serial.print("Consigne Y : "); Serial.print( int(consigneY) ); Serial.print(" ");
    Serial.print("Consigne Z : "); Serial.print( int(consigneZ) ); Serial.print(" "); Serial.println();
    */
    cmpt = 0;
  }
  
  cmpt++;

  delay(5);
}
