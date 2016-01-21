#include <Wire.h>
#include <EEPROM.h>

#include "types.h"
#include "parameters.h"
#include "configuration.h"
#include "registers_map.h"

/* Permet d'activer ou de desactiver le mode debug et les prints */
#define DEBUG                 1

#define PRINT_RAW_VALUES      0     /* Permet d'afficher les valeurs brutes du capteur */
#define PRINT_YPR_VALUES      0     /* Permet d'afficher les valeurs de angles Yaw, Pitch, Roll */

#define PRINT_PID_COMPUTING   0     /* Permet d'afficher les composants du calcul du PID */
#define AXE_TO_PRINT          0     /* Permet de selectionner l'axe a afficher pour le PID (0 : Yaw, 1 : Pitch, 2 : Roll) */

#define LOCK_YAW              0

/* --------------------------------------------------------------- */
/* ---------------------- Variables partagess -------------------- */
/* --------------------------------------------------------------- */

/* Active ou desactive les axes correspondants */
int enableYaw   = 1;
int enablePitch = 1;
int enableRoll  = 1;

/* Parametres pour la frequence des interruptions */
long PIT_LDVAL_freq[4];
long PIT_LDVAL_value[4];

/* Variables pour la gestion des donnes recues depuis la console de configuration */
int nbOctets = 0;
char data[32];

float offset[3] = {0.0, 0.0, 0.0};

/* Lookup table pour la commande des moteurs */
/* Les moteurs utilises contiennent 7 trinomes d'enroulements */
const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

/* Taille de la lookup table doit etre calculee au demarrage */
int sineArraySize;

/* Plage de valeurs pour 1 enroulement de trinome d'enroulements. (Taille de la lookup table divise par 3) */
int phaseShift;

/* Structure pour le traitement de l'asservissement PID */
t_PID ypr[3];

/* Structure pour la gestion de chaque moteur brushless */
t_motor brushless[3];

/* Structure pour la gestion du capteur */
t_IMU capteurCam;

/* Integration pour le filtre de Madgwick */
uint32_t lastUpdate = 0, Now = 0;
float deltat = 0.0;

/* Variables globales pour l'algorithme de Madgwick */
float GyroMeasError = PI * (40.0f / 180.0f);
float GyroMeasDrift = PI * (0.0f  / 180.0f);
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

void setup()
{
  configInputsOutputs();

  configMotors();
  
  /* Demarrage et initialisation du capteur */
  delay(2000);
  
  Wire.begin();
  Serial.begin(115200);
  
  /* Variable locale */
  int calibrationDone;
  
  if (testComCapteur(MPU9250_CAM_ADDRESS) == 0)
  {
    initMPU9250(MPU9250_CAM_ADDRESS);
    initAK8963(MPU9250_CAM_ADDRESS, &capteurCam);
  }
  else
  {
    #if (DEBUG)
    Serial.println("Erreur communication i2c");
    #endif
    while(1);
  }

  /* Permet d'effectuer la calibration du capteur si elle n'a pas ete effectuee */
  calibrationDone = checkCalibration();

  if (calibrationDone != 1)
  {
    /* Calibration du gyroscope et de l'accelerometre du capteur */
    gyroAccCalibration( MPU9250_CAM_ADDRESS,
                        ADDR_GYRO_CAM_BIAS, ADDR_ACC_CAM_BIAS,
                        &capteurCam);
    
    /* Calibration du magnetometre du capteur de la camera */
    magCalibration( MPU9250_CAM_ADDRESS,
                    ADD_MAG_CAM_BIAS, ADD_MAG_CAM_SCALE,
                    &capteurCam);
    
    calibrationDone = 1;
    EEPROM.put(FIRST_CALIBRATION_FLAG, calibrationDone);
  }

  /* Chargement des parametres du gyroscope et de l'accelerometre */
  loadGyroAccCalibrationBias( ADDR_GYRO_CAM_BIAS, ADDR_ACC_CAM_BIAS,
                              capteurCam.gyroBias, capteurCam.accBias);

  /* Chargement des parametres du magnetometre */
  loadMagCalibrationData( ADD_MAG_CAM_BIAS, ADD_MAG_CAM_SCALE,
                          capteurCam.magBias, capteurCam.magScale);

  #if (DEBUG)
    printSensorValues(capteurCam);
  #endif

  getFirstOrientations(MPU9250_CAM_ADDRESS, AK8963_ADDRESS, &capteurCam);

  loadInitialParameters();

  configPID();

  //loadParameters();

  configInterrupts();

  deltat = 1 / (float)MADGWICK_FREQUENCY;
}

void loop()
{
  //plot(capteurCam.ypr[0], capteurCam.ypr[1], capteurCam.ypr[2], ypr[0].consigne, ypr[1].consigne, ypr[2].consigne);

  receiveDataFromConsole();

  #if (PRINT_PID_COMPUTING)
  
  Serial.print(ypr[AXE_TO_PRINT].consigne);
  Serial.print(" = ");
  Serial.print(ypr[AXE_TO_PRINT].kp);
  Serial.print(" * ");
  Serial.print(ypr[AXE_TO_PRINT].erreur);
  Serial.print(" + ");
  Serial.print(ypr[AXE_TO_PRINT].ki);
  Serial.print(" * ");
  Serial.print(ypr[AXE_TO_PRINT].sommeErreurs);
  Serial.print(" + ");
  Serial.print(ypr[AXE_TO_PRINT].kd);
  Serial.print(" * ");
  Serial.println(ypr[AXE_TO_PRINT].variationErreur);
  
  #endif

}

/* --------------------------------------------------------------- */
/* ----------------- Gestion de l'interruption 0 ----------------- */
/* --------------------------------------------------------------- */
void pit0_isr
(
  /* Ne prend pas de parametres */
)
{
  /* Recuperation des donnees brutes du capteur */
  getOrientation(MPU9250_CAM_ADDRESS, &capteurCam);

  /* Conversion de l'orientation sous forme de representation Euler */
  getYawPitchRoll(&capteurCam);
  
  PIT_TFLG0 = 1;
}

/* --------------------------------------------------------------- */
/* ----------------- Gestion de l'interruption 1 ----------------- */
/* --------------------------------------------------------------- */
void pit1_isr
(
  /* Ne prend pas de parametres */
)
{
  /* Application du PID pour calculer la consigne a appliquer a la PO */
  computeOrder(capteurCam.ypr);

  /* Commande des moteurs */
  updatePO();

  PIT_TFLG1 = 1;
}

/* --------------------------------------------------------------- */
/* ----------------- Gestion de l'interruption 2 ----------------- */
/* --------------------------------------------------------------- */
void pit2_isr
(
  /* Ne prend pas de parametres */
)
{
  /* Estimation de l'orientation avec le filtre de Madgwick */
  MadgwickQuaternionUpdate( capteurCam.acc[0], capteurCam.acc[1], capteurCam.acc[2],
                            capteurCam.gyro[0] * DEG_TO_RAD, capteurCam.gyro[1] * DEG_TO_RAD, capteurCam.gyro[2] * DEG_TO_RAD,
                            //capteurCam.mag[1],  capteurCam.mag[0], capteurCam.mag[2],
                            //capteurCam.mag[1],  capteurCam.mag[0], -capteurCam.mag[2],
                            capteurCam.q);

  PIT_TFLG2 = 1;
}

/* --------------------------------------------------------------- */
/* -------- Permet de tracer les donnees d'asservissement -------- */
/* --------------------------------------------------------------- */
void plot
(
  float cpt_yaw,    /* Orientation sur Yaw */
  float cpt_pitch,  /* Orientation sur Pitch */
  float cpt_roll,   /* Orientation sur Roll */
  int cmd_yaw,      /* Commande appliquee sur Yaw */
  int cmd_pitch,    /* Commande appliquee sur Pitch */
  int cmd_roll      /* Commande appliquee sur Roll */
)
{
  char buffer_part_1[32];
  char buffer_part_2[32];
  char buffer[64];
  char cpt_yaw_str[16];
  char cpt_pitch_str[16];
  char cpt_roll_str[16];

  dtostrf(cpt_yaw, 4, 3, cpt_yaw_str);
  dtostrf(cpt_pitch, 4, 3, cpt_pitch_str);
  dtostrf(cpt_roll, 4, 3, cpt_roll_str);

  strcpy(buffer_part_1, cpt_yaw_str);
  strcat(buffer_part_1, " ");
  strcat(buffer_part_1, cpt_pitch_str);
  strcat(buffer_part_1, " ");
  strcat(buffer_part_1, cpt_roll_str);
  
  sprintf(buffer_part_2, "%i %i %i", cmd_yaw, cmd_pitch, cmd_roll);
  
  strcpy(buffer, buffer_part_1);
  strcat(buffer, " ");
  strcat(buffer, buffer_part_2);
  
  Serial.println(buffer);
}

/* --------------------------------------------------------------- */
/* -------- Permet de recevoir les parametres de la console ------ */
/* ------------- pour les parametres de configuration ------------ */
/* --------------------------------------------------------------- */
void receiveDataFromConsole
(
  // Ne prend pas de paramètres
)
{
  char dataTmp;
  int i;
  
  if (Serial.available() > 0)
  {
    dataTmp = Serial.read();
    
    if (dataTmp != ';')
    {
      data[nbOctets] = dataTmp;
      nbOctets++;
    }
    else
    {
      data[nbOctets] = dataTmp;
      applyParameters(data);
      
      for (i = 0; i < 32; i++)  data[i] = 0;
      nbOctets = 0;
    }
  }
}

/* --------------------------------------------------------------- */
/* -------- Permet l'application des parametres mis a jour ------- */
/* --------------------------------------------------------------- */
void applyParameters
(
  char * parameter
)
{
  /* Variables locales */
  int index = 0;
  int indexKey = 0;
  int indexValue = 0;
  char key[16];
  char value[16];
  float fValue;

  while (parameter[index] != ' ')
  {
    if (parameter[index] != 0x0A)
    {
      key[indexKey] = parameter[index];
      indexKey++;
    }
    index++;
  }
  key[indexKey] = '\0';
  
  while (parameter[index] != ';')
  {
    value[indexValue] = parameter[index];
    index++;
    indexValue++;
  }
  value[indexValue] = '\0';

  /*
  Serial.print("Key : ");
  Serial.print(key);
  Serial.print(", Value : ");
  Serial.println(value);
  */

  fValue = atof(value);

  /* Activation / desactivation de l'axe ROLL */
  if (strcmp(key, ENABLE_ROLL_AXIS) == 0)
  {
    if (fValue > 0.5)   digitalWrite(brushless[2].enableMotor, HIGH);
    else                digitalWrite(brushless[2].enableMotor, LOW);
  }
  
  /* Activation / desactivation de l'axe PITCH */
  if (strcmp(key, ENABLE_PITCH_AXIS) == 0)
  {
    if (fValue > 0.5)   digitalWrite(brushless[1].enableMotor, HIGH);
    else                digitalWrite(brushless[1].enableMotor, LOW);
  }

  /* Activation / desactivation de l'axe YAW */
  if (strcmp(key, ENABLE_YAW_AXIS) == 0)
  {
    if (fValue > 0.5)   digitalWrite(brushless[0].enableMotor, HIGH);
    else                digitalWrite(brushless[0].enableMotor, LOW);
    
    storeParameters();
  }

  /* Modification des valeurs de Kp, Ki, Kd pour Yaw, Pitch, Roll */
  if (strcmp(key, KP_ROLL) == 0)              ypr[2].kp = fValue;
  if (strcmp(key, KI_ROLL) == 0)              ypr[2].ki = fValue;
  if (strcmp(key, KD_ROLL) == 0)              ypr[2].kd = fValue;
  if (strcmp(key, KP_PITCH) == 0)             ypr[1].kp = fValue;
  if (strcmp(key, KI_PITCH) == 0)             ypr[1].ki = fValue;
  if (strcmp(key, KD_PITCH) == 0)             ypr[1].kd = fValue;
  if (strcmp(key, KP_YAW) == 0)               ypr[0].kp = fValue;
  if (strcmp(key, KI_YAW) == 0)               ypr[0].ki = fValue;
  if (strcmp(key, KD_YAW) == 0)               ypr[0].kd = fValue;

  /* Modification de la frequence d'acquisition des donnees du capteur */
  if (strcmp(key, FREQ_CPT) == 0)
  {
    PIT_LDVAL_freq[0] = (int)fValue;
    configFreqInterrupt();
  }

  /* Modification de la frequence de commande des moteurs */
  if (strcmp(key, FREQ_MOTORS) == 0)
  {
    PIT_LDVAL_freq[2] = (int)fValue;
    configFreqInterrupt();
  }

  /* Modification de la frequence de la boucle d'asservissement */
  if (strcmp(key, FREQ_PID) == 0)
  {
    PIT_LDVAL_freq[1] = (int)fValue;
    configFreqInterrupt();
  }

  /* Modification de la frequence de la boucle du filtre de Madgwick */
  if (strcmp(key, FREQ_MADGWICK) == 0)
  {
    PIT_LDVAL_freq[3] = (int)fValue;
    configFreqInterrupt();
  }

  /* Demande d'enregistrement des parametres */
  if (strcmp(key, RECORD_PARAMETERS) == 0)   storeParameters();
  
}

/* --------------------------------------------------------------- */
/* --- Permet l'enregistrement des parametres d'asservissement --- */
/* --------------------------------------------------------------- */
void storeParameters
(
  // Ne prend pas de paramètres
)
{
  Serial.println("Enregistrement");
  
  /* Variable locale */
  int i;
  
  /* Enregistrement des parametres Kp pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.put( (ADDR_KP_PARAMETERS + i * 4), ypr[i].kp);
  }

  /* Enregistrement des parametres Ki pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.put( (ADDR_KI_PARAMETERS + i * 4), ypr[i].ki);
  }

  /* Enregistrement des parametres Kd pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.put( (ADDR_KD_PARAMETERS + i * 4), ypr[i].kd);
  }

  /* Enregistrement des valeurs de frequence des interruptions */
  for (i = 0; i < 3; i++)
  {
    EEPROM.put( (ADDR_INTERRUPT_VALUES + i * 4), PIT_LDVAL_freq[i]);
  }
}

/* --------------------------------------------------------------- */
/* ---- Permet le chargement des parametres d'asservissement ----- */
/* --------------------------------------------------------------- */
void loadParameters
(
  // Ne prend pas de paramètres
)
{
  /* Variable locale */
  int i;
  
  /* Enregistrement des parametres Kp pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.get( (ADDR_KP_PARAMETERS + i * 4), ypr[i].kp);
  }

  /* Enregistrement des parametres Ki pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.get( (ADDR_KI_PARAMETERS + i * 4), ypr[i].ki);
  }

  /* Enregistrement des parametres Kd pour Yaw, Pitch, Roll */
  for (i = 0; i < 3; i++)
  {
    EEPROM.get( (ADDR_KD_PARAMETERS + i * 4), ypr[i].kd);
  }

  /* Enregistrement des valeurs de frequence des interruptions */
  for (i = 0; i < 3; i++)
  {
    //EEPROM.get( (ADDR_INTERRUPT_VALUES + i * 4), PIT_LDVAL_freq[i]);
  }

  Serial.print("Yaw ===> Kp = "); Serial.print(ypr[0].kp, 5); Serial.print(", Ki = "); Serial.print(ypr[0].ki, 5); Serial.print(", Kd = "); Serial.println(ypr[0].kd, 5);
  Serial.print("Pitch => Kp = "); Serial.print(ypr[1].kp, 5); Serial.print(", Ki = "); Serial.print(ypr[1].ki, 5); Serial.print(", Kd = "); Serial.println(ypr[1].kd, 5);
  Serial.print("Roll ==> Kp = "); Serial.print(ypr[2].kp, 5); Serial.print(", Ki = "); Serial.print(ypr[2].ki, 5); Serial.print(", Kd = "); Serial.println(ypr[2].kd, 5);
  Serial.print("Frequence capteurs : ");          Serial.print(PIT_LDVAL_freq[0]); Serial.println(" Hz");
  Serial.print("Frequence PID et moteurs : ");    Serial.print(PIT_LDVAL_freq[1]); Serial.println(" Hz");
  Serial.print("Frequence Madgwick : ");          Serial.print(PIT_LDVAL_freq[2]); Serial.println(" Hz");
}


/* --------------------------------------------------------------- */
/* --------- Permet de stabiliser le filtre de Madgwick ---------- */
/* --------------------------------------------------------------- */
void getFirstOrientations
(
  byte address,     /* Adresse i2c du capteur : Gyroscope et accelerometere */
  byte addressMag,  /* Adresse i2c du magnetometre */
  t_IMU * capteur   /* Reference sur le capteur */
)
{
  /* Variable locale */
  int i;
  
  for (i = 0; i < 400; i++)
  {
    /* Orientation de la camera */
    getOrientation(address, capteur);

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;
    MadgwickQuaternionUpdate( capteur->acc[0], capteur->acc[1], capteur->acc[2],
                              capteur->gyro[0] * DEG_TO_RAD, capteur->gyro[1] * DEG_TO_RAD, capteur->gyro[2] * DEG_TO_RAD,
                              //capteurCam.mag[1],  capteurCam.mag[0], capteurCam.mag[2],
                              //capteurCam.mag[1],  capteurCam.mag[0], -capteurCam.mag[2],
                              capteur->q);
                   
    getYawPitchRoll(capteur);

  }

  /* Enregistrement de l'offset initial pour l'axe Yaw */
  offset[0] = capteur->ypr[0];
  offset[1] = 90;
}

/* --------------------------------------------------------------- */
/* ---- Verifie si la calibration du capteur a ete effectuee ----- */
/* --------------------------------------------------------------- */
int checkCalibration
(
  /* Ne prend pas de parametres */
)
{
  /* Variable locale */
  int firstCalibrationDone;
  
  EEPROM.get(FIRST_CALIBRATION_FLAG ,firstCalibrationDone);

  return firstCalibrationDone;
}

/* --------------------------------------------------------------- */
/* --- Routine de calibration de l'accelerometre et gyroscope ---- */
/* --------------------------------------------------------------- */
void gyroAccCalibration
(
  char address,         /* Adresse i2c du capteur */
  char addressGyroBias, /* Adresse EEPROM pour l'offset du gyroscope */
  char addressAccBias,  /* Adresse EEPROM pour l'offset de l'accelerometre */
  t_IMU * capteur       /* Reference sur le capteur */
)
{
  /* Variables locales */
  float accError[3] = {0.0f, 0.0f, 0.0f};
  float gyroError[3] = {0.0f, 0.0f, 0.0f};
  int i, j;
  t_IMU capteurWithNoBias;

  /* Mise a zero de l'offset */
  for (i = 0; i < 3; i++)
  {
    capteurWithNoBias.accBias[i] = 0.0;
    capteurWithNoBias.gyroBias[i] = 0.0;
  }

  #if (DEBUG)
  Serial.println("Etalonnage du gyroscope et accelerometre...");
  #endif
  
  for (i = 0; i < 1000; i++) 
  {
    getOrientationWithoutMag(address, &capteurWithNoBias);
  
    for (j = 0; j < 3; j++)
    {
      accError[j] += capteurWithNoBias.acc[j];
      gyroError[j] += capteurWithNoBias.gyro[j];
    }
    
    delay(2);
  }

  for (i = 0; i < 3; i++)
  {
    capteur->accBias[i] = accError[i] / 1000.0f;
    capteur->gyroBias[i] = gyroError[i] / 1000.0f;
  }
  capteur->accBias[2] = -(1.0 - capteur->accBias[2]);

  storeGyroAccCalibrationBias(addressGyroBias, addressAccBias, capteur->gyroBias, capteur->accBias);

  #if (DEBUG)
    Serial.println("Etalonnage acc / gyro termine !");
    Serial.print("accBiasX : ");
    Serial.print(capteur->accBias[0], 5);
    Serial.print(", accBiasY : ");
    Serial.print(capteur->accBias[1], 5);
    Serial.print(", accBiasZ : ");
    Serial.print(capteur->accBias[2], 5);
    Serial.print(" -- gyroBiasX : ");
    Serial.print(capteur->gyroBias[0], 5);
    Serial.print(", gyroBiasY : ");
    Serial.print(capteur->gyroBias[1], 5);
    Serial.print(", gyroBiasZ : ");
    Serial.println(capteur->gyroBias[2], 5);
  #endif
  
}

/* --------------------------------------------------------------- */
/* ----------- Routine de calibration du magnetometre  ----------- */
/* --------------------------------------------------------------- */
void magCalibration
(
  byte address,         /* Adresse i2c du capteur */
  byte addressMagBias,  /* Adresse EEPROM pour l'offset du magnetometre */
  byte addressMagScale, /* Adresse EEPROM pour l'echelle du magnetometre */
  t_IMU * capteur       /* Reference sur le capteur */
)
{
  /* Variables locales */
  int16_t magRaw[3] = {0, 0, 0};
  float mag[3] = {0.0f, 0.0f, 0.0f};
  float minMag[3] = {0.0f, 0.0f, 0.0f};
  float maxMag[3] = {0.0f, 0.0f, 0.0f};
  int validCalibration = 0, i;

  #if (DEBUG)
  Serial.println("Etalonnage du magnemotre...");
  #endif

  do {
    readMagData(magRaw, address);

    for (i = 0; i < 3; i++)
    {
      mag[i] = (float)magRaw[i];

      /* Determination des valeurs min et max */
      if (mag[i] < minMag[i]) minMag[i] = mag[i];
      if (mag[i] > maxMag[i]) maxMag[i] = mag[i];
    }

    Serial.print("X : "); Serial.print(minMag[0]); Serial.print(" - "); Serial.print(maxMag[0]);
    Serial.print(" === Y : "); Serial.print(minMag[1]); Serial.print(" - "); Serial.print(maxMag[1]);
    Serial.print(" === Z : "); Serial.print(minMag[2]); Serial.print(" - "); Serial.println(maxMag[2]);
    delay(10);

    /* Permet d'arreter la calibration lorsque le systeme est immobile */
    getOrientationWithoutMag(address, capteur);
    if (  (capteur->gyro[0] <= 2.0 && capteur->gyro[0] >= -2.0) && 
          (capteur->gyro[1] <= 2.0 && capteur->gyro[1] >= -2.0) &&
          (capteur->gyro[2] <= 2.0 && capteur->gyro[2] >= -2.0) )
    {
      validCalibration++;
      delay(10);
    }
    
  } while (validCalibration < 200); /* 200 * 10 ms = 2 secondes stable pour terminer la calibration */

  capteur->magBias[0] = (maxMag[0] + minMag[0]) / 2;
  capteur->magBias[1] = (maxMag[1] + minMag[1]) / 2;
  capteur->magBias[2] = (maxMag[2] + minMag[2]) / 2;

  calculateGyroScale(maxMag, minMag, capteur->magScale);

  storeMagCalibrationData(addressMagBias, addressMagScale, capteur->magBias, capteur->magScale);

  #if (DEBUG)
    Serial.println("Etalonnage du magnetometre termine !");
    Serial.println("minX : maxX : minY : maxY : minZ : maxZ -- magBiasX : magBiasY : magBiasZ -- magScaleX : magScaleY : magScaleZ");
    Serial.print(minMag[0]);
    Serial.print(" : ");
    Serial.print(maxMag[0]);
    Serial.print(" : ");
    Serial.print(minMag[1]);
    Serial.print(" : ");
    Serial.print(maxMag[1]);
    Serial.print(" : ");
    Serial.print(minMag[2]);
    Serial.print(" : ");
    Serial.print(maxMag[2]);
    Serial.print(" -- ");
    Serial.print(capteur->magBias[0]);
    Serial.print(" : ");
    Serial.print(capteur->magBias[1]);
    Serial.print(" : ");
    Serial.print(capteur->magBias[2]);
    Serial.print(" -- ");
    Serial.print(capteur->magScale[0]);
    Serial.print(" : ");
    Serial.print(capteur->magScale[1]);
    Serial.print(" : ");
    Serial.println(capteur->magScale[2]);
  #endif
}

/* --------------------------------------------------------------- */
/* --------- Enregistrement des donnees de calibration  ---------- */
/* ----- de l'accelerommetre et gyroscope depuis la EEPROM  ------ */
/* --------------------------------------------------------------- */
void storeGyroAccCalibrationBias
(
  byte addressGyroBias,   /* Adresse EEPROM de l'offset du gyroscope */
  byte addressAccBias,    /* Adresse EEPROM de l'offset de l'accelerometre */
  float * gyroBias,       /* Reference sur l'offset du gyroscope */
  float * accBias         /* Reference sur l'offset de l'accelerometre */
)
{
  for (int i = 0; i < 3; i++)
  {
    EEPROM.put( (addressGyroBias + i * 4), gyroBias[i]);
    EEPROM.put( (addressAccBias + i * 4), accBias[i]);
  }
}

/* --------------------------------------------------------------- */
/* ----------- Chargement des donnees de calibration  ------------ */
/* ----- de l'accelerommetre et gyroscope depuis la EEPROM  ------ */
/* --------------------------------------------------------------- */
void loadGyroAccCalibrationBias
(
  byte addressGyroBias,   /* Adresse EEPROM de l'offset du gyroscope */
  byte addressAccBias,    /* Adresse EEPROM de l'offset de l'accelerometre */
  float * gyroBias,       /* Reference sur l'offset du gyroscope */
  float * accBias         /* Reference sur l'offset de l'accelerometre */
)
{
  for (int i = 0; i < 3; i++)
  {
    EEPROM.get( (addressGyroBias + i * 4), gyroBias[i]);
    EEPROM.get( (addressAccBias + i * 4), accBias[i]);
  }
}

/* --------------------------------------------------------------- */
/* --------- Enregistrement des donnees de calibration  ---------- */
/* -------------- du magnetometre depuis la EEPROM  -------------- */
/* --------------------------------------------------------------- */
void storeMagCalibrationData
(
  byte addressMagBias,    /* Adresse EEPROM de l'offset du magnetometre */
  byte addressMagScale,   /* Adresse EEPROM de l'echelle du magnetometre */
  float * magBias,        /* Reference sur l'offset du magnetometre */
  float * magScale        /* Reference sur l'echelle du magnetometre */
)
{
  for (int i = 0; i < 3; i++)
  {
    EEPROM.put(addressMagBias + i * 4, magBias[i]);
    EEPROM.put(addressMagScale + i * 4, magScale[i]);
  }
}

/* --------------------------------------------------------------- */
/* ----------- Chargement des donnees de calibration  ------------ */
/* -------------- du magnetometre depuis la EEPROM  -------------- */
/* --------------------------------------------------------------- */
void loadMagCalibrationData
(
  byte addressMagBias,    /* Adresse EEPROM de l'offset du magnetometre */
  byte addressMagScale,   /* Adresse EEPROM de l'echelle du magnetometre */
  float * magBias,        /* Reference sur l'offset du magnetometre */
  float * magScale        /* Reference sur l'echelle du magnetometre */
)
{
  for (int i = 0; i < 3; i++)
  {
    EEPROM.get(addressMagBias + i * 4, magBias[i]);
    EEPROM.get(addressMagScale + i * 4, magScale[i]);
  }
}

/* --------------------------------------------------------------- */
/* ---------- Test de la communication avec le capteur  ---------- */
/* --------------------------------------------------------------- */
int testComCapteur
(
  byte address  /* Adresse i2c du capteur */
)
{
  /* Variable locale */
  byte c;

  /* Lecture d'un registre pour tester la communication */
  c = readByte(address, WHO_AM_I_MPU9250);

  if (c == 0x71)  return 0;   /* Communication OK */
  else            return -1;  /* Communication NOK */
}

/* --------------------------------------------------------------- */
/* ----------- Calcul de l'echelle pour le gyroscope  ------------ */
/* --------------------------------------------------------------- */
void calculateGyroScale
(
  float * magMax,   /* Valeurs max du magnetometre */
  float * magMin,   /* Valeurs min du magnetometre */
  float * magScale  /* Echelle du magnetometre */
)
{
  /* Variables locales */
  int i;
  float rawMagScale[3], avgMagScale;

  for (i = 0; i < 3; i++)     rawMagScale[i] = (magMax[i] - magMin[i]) / 2;
  
  avgMagScale = (rawMagScale[0] + rawMagScale[1] + rawMagScale[2]) / 3;

  for (i = 0; i < 3; i++)     magScale[i] = avgMagScale / rawMagScale[i];

}

/* --------------------------------------------------------------- */
/* -------------------- Initialisation des PID  ------------------ */
/* --------------------------------------------------------------- */
void configPID
(
  /* Ne prend pas de parametres */
)
{
  /* Variable locale */
  int i;
  float nbMotorStep;

  /* Initialisation du PID pour le demarrage */
  for (i = 0; i < 3; i++)
  {
    nbMotorStep = (sineArraySize + 1) * NB_ELEC_BLOCS;
    ypr[i].sommeErreurs = (nbMotorStep / 2.0) / ypr[i].ki;
  }
}

/* --------------------------------------------------------------- */
/* ------------- Configuration des entres / sorties  ------------- */
/* --------------------------------------------------------------- */
void configInputsOutputs
(
  /* Ne prend pas de parametres */
)
{
  /* Variables locales */
  int i, j;
  
  /* Affectation des numeros de PIN du microcontroleur pour les 3 moteurs brushless */
  brushless[0].input[0] = YAW_MOTOR_P1;
  brushless[0].input[1] = YAW_MOTOR_P2;
  brushless[0].input[2] = YAW_MOTOR_P3;
  brushless[0].enableMotor = YAW_MOTOR_EN;

  brushless[1].input[0] = PITCH_MOTOR_P1;
  brushless[1].input[1] = PITCH_MOTOR_P2;
  brushless[1].input[2] = PITCH_MOTOR_P3;
  brushless[1].enableMotor = PITCH_MOTOR_EN;

  brushless[2].input[0] = ROLL_MOTOR_P1;
  brushless[2].input[1] = ROLL_MOTOR_P2;
  brushless[2].input[2] = ROLL_MOTOR_P3;
  brushless[2].enableMotor = ROLL_MOTOR_EN;

  /* Configuration en sortie pour les 3 signaux PWM et le signal d'activation du moteur */
  for (i = 0; i < 3; i++)
  {
    pinMode(brushless[i].enableMotor, OUTPUT);

    for (j = 0; j < 3; j++)     pinMode(brushless[i].input[j], OUTPUT);
    
  }
}

/* --------------------------------------------------------------- */
/* ----------------- Configuration des moteurs  ------------------ */
/* --------------------------------------------------------------- */
void configMotors
(
  /* Ne prend pas de parametres */
)
{
  /* Variables locales */
  int i, j;

  /* Initilisation de la taille de la lookup table */
  sineArraySize = sizeof(pwmSin) / sizeof(int);
  sineArraySize--;
  
  phaseShift = sineArraySize / 3;

  for (i = 0; i < 3; i++)
  {
    /* Initilialisation de la consigne et de l'image a 180 degres */
    brushless[i].consigne = ((sineArraySize + 1) * NB_ELEC_BLOCS) / 2;
    brushless[i].image = ((sineArraySize + 1) * NB_ELEC_BLOCS) / 2;
    ypr[i].consigne = ((sineArraySize + 1) * NB_ELEC_BLOCS) / 2;

    /* Initilisation de la valeur de chaque step d'enroulement */
    brushless[i].currentStep[0] = 0;
    brushless[i].currentStep[1] = brushless[i].currentStep[0] + phaseShift;
    brushless[i].currentStep[2] = brushless[i].currentStep[1] + phaseShift;

    /* Configuration de la frequence des PWM pour eviter le sifflement des moteurs */
    for (j = 0; j < 3; j++)     analogWriteFrequency(brushless[i].input[j], PWM_FREQUENCY);
  }

  if (enableYaw == 1)   digitalWrite(brushless[0].enableMotor, HIGH);
  else                  digitalWrite(brushless[0].enableMotor, LOW);

  if (enablePitch == 1) digitalWrite(brushless[1].enableMotor, HIGH);
  else                  digitalWrite(brushless[1].enableMotor, LOW);

  if (enableRoll == 1) digitalWrite(brushless[2].enableMotor, HIGH);
  else                  digitalWrite(brushless[2].enableMotor, LOW);

  /* Configuration du sens de rotation pour chaque moteur */
  brushless[0].sensRotation = SENS_MOTOR_YAW;
  brushless[1].sensRotation = SENS_MOTOR_PITCH;
  brushless[2].sensRotation = SENS_MOTOR_ROLL;
}

/* --------------------------------------------------------------- */
/* ------------------- Commande des moteurs  --------------------- */
/* --------------------------------------------------------------- */
void updatePO
(
  /* Ne prend pas de parametres */
)
{
  /* Variables locales */
  int i, j;

  for (i = 0; i < 3; i++)
  {
    /* Mise a l'echelle de la consigne calculee par l'asservissement */
    brushless[i].consigne = ypr[i].consigne;

    /* Commande des moteurs */
    for (j = 0; j < 3; j++)     analogWrite(brushless[i].input[j], pwmSin[ brushless[i].currentStep[j] ]);
    
    /* Gestion de la commande des moteurs en fonction de la consigne et de la position actuelle */
    if ( !(i == 0 && LOCK_YAW) ) {
      if (brushless[i].consigne > brushless[i].image) brushless[i].increment = 1;        /* Compensation positive */
      else if (brushless[i].consigne < brushless[i].image) brushless[i].increment = -1;  /* Compensation negative */
      else brushless[i].increment = 0;                                                   /* Pas de compensation */
    }
    else brushless[0].increment = 0;
    
    /* Mise a jour de la position demandee en fonction de l'increment precedemment calcule */
    for (j = 0; j < 3; j++)     brushless[i].currentStep[j] = brushless[i].currentStep[j] + brushless[i].increment;
    
    /* Mise a jour de l'image */
    brushless[i].image = brushless[i].image + brushless[i].increment;

    /* Gestion du dépassement de la lookup table */
    for (j = 0; j < 3; j++)
    {
      if (brushless[i].currentStep[j] > sineArraySize) brushless[i].currentStep[j] = 0;
      if (brushless[i].currentStep[j] < 0) brushless[i].currentStep[j] = sineArraySize;
    }
  }
}

/* --------------------------------------------------------------- */
/* ------------- Calcul de la consigne des moteurs  -------------- */
/* --------------------- Asservissement PID  --------------------- */
/* --------------------------------------------------------------- */
void computeOrder
(
  float erreur[3]   /* Erreur d'orientation en degres sur les 3 axes */
)
{
  /* Variable locale */
  int i;
  float erreurToCompute;

  for (i = 0; i < 3; i++)
  {
    if ( (i == 0 && enableYaw == 1) || (i == 1 && enablePitch == 1)  || (i == 2 && enableRoll == 1) )
    {
      /* Prise en compte de l'offset */
      erreurToCompute = erreur[i] - offset[i];
      erreurToCompute = erreurToCompute * (float)brushless[i].sensRotation;

      /* Garde uniquement 1 chiffre apres la firgule */
      erreurToCompute = (int)(erreurToCompute * 10);
      erreurToCompute = erreurToCompute / 10;
      
      ypr[i].erreur = erreurToCompute;
      ypr[i].sommeErreurs += ypr[i].erreur;
      ypr[i].variationErreur = ypr[i].erreur - ypr[i].erreurPrecedente;

      /* Calcul PID */
      ypr[i].consigne = ypr[i].kp * ypr[i].erreur + ypr[i].ki * ypr[i].sommeErreurs + ypr[i].kd * ypr[i].variationErreur;
      
      ypr[i].erreurPrecedente = ypr[i].erreur;

      /* Securite pour limiter la valeur de consigne a appliquer aux moteurs */
      if (ypr[i].consigne > ((sineArraySize + 1) * NB_ELEC_BLOCS) )       ypr[i].consigne = (sineArraySize + 1) * NB_ELEC_BLOCS;
      if (ypr[i].consigne < -((sineArraySize + 1) * NB_ELEC_BLOCS) )      ypr[i].consigne = -((sineArraySize + 1) * NB_ELEC_BLOCS);
    }
  }
}

/* --------------------------------------------------------------- */
/* ------------- Chargement des parametres initiaux  ------------- */
/* --------------------------------------------------------------- */
void loadInitialParameters
(
  // Ne prend pas de parametres
)
{
  ypr[0].kp = KP_Y;
  ypr[0].ki = KI_Y;
  ypr[0].kd = KD_Y;
  ypr[1].kp = KP_P;
  ypr[1].ki = KI_P;
  ypr[1].kd = KD_P;
  ypr[2].kp = KP_R;
  ypr[2].ki = KI_R;
  ypr[2].kd = KD_R;

  PIT_LDVAL_freq[0] = UPDATE_CPT_FREQUENCY;
  PIT_LDVAL_freq[1] = PID_FREQUENCY;
  PIT_LDVAL_freq[2] = MADGWICK_FREQUENCY;
}

/* --------------------------------------------------------------- */
/* ----- Configuration de la frequence des interruptions  -------- */
/* --------------------------------------------------------------- */
void configFreqInterrupt
(
  // Ne prend pas de parametres
)
{
  /* 50 000 000 = 1 / seconde */
  PIT_LDVAL_value[0] = (int)(50000000 / PIT_LDVAL_freq[0]);
  PIT_LDVAL_value[1] = (int)(50000000 / PIT_LDVAL_freq[1]);
  PIT_LDVAL_value[2] = (int)(50000000 / PIT_LDVAL_freq[2]);

  PIT_LDVAL0 = PIT_LDVAL_value[0];
  PIT_LDVAL1 = PIT_LDVAL_value[1];
  PIT_LDVAL2 = PIT_LDVAL_value[2];
}

/* --------------------------------------------------------------- */
/* -------------- Configuration des interruptions  --------------- */
/* ----------------------- et activation  ------------------------ */
/* --------------------------------------------------------------- */
void configInterrupts
(
  /* Ne prend pas de parametres */
)
{
  PIT_LDVAL_value[0] = (int)(50000000 / PIT_LDVAL_freq[0]);
  PIT_LDVAL_value[1] = (int)(50000000 / PIT_LDVAL_freq[1]);
  PIT_LDVAL_value[2] = (int)(50000000 / PIT_LDVAL_freq[2]);
  PIT_LDVAL_value[3] = (int)(50000000 / PIT_LDVAL_freq[3]);
  
  /* Activation */
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;

  /* Interruption pour le rafraichissement des donnees du capteur */
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  PIT_LDVAL0 = PIT_LDVAL_value[0];            
  PIT_TCTRL0 = 0x02;
  PIT_TCTRL0 |= 0x01;
  PIT_TFLG0 |= 1;

  /* Interruption pour l'asservissement et commande des moteurs */
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  PIT_LDVAL1 = PIT_LDVAL_value[1];
  PIT_TCTRL1 = 0x02;
  PIT_TCTRL1 |= 0x01;
  PIT_TFLG1 |= 1;

  /* Interruption pour la boucle du filtre de Madgwick */
  NVIC_ENABLE_IRQ(IRQ_PIT_CH2);
  PIT_LDVAL2 = PIT_LDVAL_value[2];
  PIT_TCTRL2 = 0x02;
  PIT_TCTRL2 |= 0x01;
  PIT_TFLG2 |= 1;
}

/* --------------------------------------------------------------- */
/* ------ Permet d'obtenir l'orientation sur Yaw Pitch Roll ------ */
/* --------------------------------------------------------------- */
void getYawPitchRoll
(
  t_IMU * capteur   /* Reference sur le capteur */
)
{
  /* Variables locales */
  int i;

  /* Calcul de YAW a partir des quaternions */
  capteur->ypr[0] = atan2(2.0f * (capteur->q[1] * capteur->q[2] + capteur->q[0] * capteur->q[3]), capteur->q[0] * capteur->q[0] + capteur->q[1] * capteur->q[1] - capteur->q[2] * capteur->q[2] - capteur->q[3] * capteur->q[3]);
  
  /* Calcul de PITCH a partir des quaternions */
  capteur->ypr[2] = -asin(2.0f * (capteur->q[1] * capteur->q[3] - capteur->q[0] * capteur->q[2]));
  
  /* Calcul de ROLL a partir des quaternions */
  capteur->ypr[1] = atan2(2.0f * (capteur->q[0] * capteur->q[1] + capteur->q[2] * capteur->q[3]), capteur->q[0] * capteur->q[0] - capteur->q[1] * capteur->q[1] - capteur->q[2] * capteur->q[2] + capteur->q[3] * capteur->q[3]);

  /* Conversion des valeurs de Yaw Pitch Roll en degres */
  for (i = 0; i < 3; i++)     capteur->ypr[i] *= RAD_TO_DEG;

  #if (PRINT_YPR_VALUES)
    Serial.print("Yaw : "); Serial.print(capteur->ypr[0], 4);
    Serial.print(", Pitch : "); Serial.print(capteur->ypr[1], 4);
    Serial.print(", Roll : "); Serial.println(capteur->ypr[2], 4);
  #endif
}

/* --------------------------------------------------------------- */
/* ----------------- Mise a jour de l'orientation ---------------- */
/* -- des donnees de l'accelerometre, gyroscope et magnetometre -- */
/* --------------------------------------------------------------- */
void getOrientation
(
  byte address,          /* Adresse i2c du capteur */
  t_IMU * capteur             /* Reference sur le capteur */
)
{
  /* Variables locales */
  int16_t rawData[3] = {0, 0, 0};
  int i;

  /* Recuperation et traitement des donnees de l'accelerometre */
  readAccelData(rawData, address);
  for (i = 0; i < 3; i++)     capteur->acc[i] = (float)rawData[i] * (float)ACC_RESOLUTION - capteur->accBias[i];

  /* Recuperation et traitement des donnees du gyroscope */
  readGyroData(rawData, address);
  for (i = 0; i < 3; i++)     capteur->gyro[i] = (float)rawData[i] * (float)GYRO_RESOLUTION - capteur->gyroBias[i];

  /* Recuperation et traitement des donnees du magnetometre */
  readMagData(rawData, address);

  for (i = 0; i < 3; i++) {
    capteur->mag[i] = ( ((float)rawData[i] * MAG_RESOLUTION * capteur->magSensitivity[i]) - (capteur->magBias[i] * MAG_RESOLUTION * capteur->magSensitivity[i]) ) * capteur->magScale[i];
  }
  
  #if (PRINT_RAW_VALUES)
    /*
    Serial.print("X-acceleration: "); Serial.print(capteur->acc[0], 4); Serial.print(" g\t\t");
    Serial.print("Y-acceleration: "); Serial.print(capteur->acc[1], 4); Serial.print(" g\t\t");
    Serial.print("Z-acceleration: "); Serial.print(capteur->acc[2], 4); Serial.print(" g\t\t -- \t\t");
    
    Serial.print("X-gyro rate: "); Serial.print(capteur->gyro[0], 4); Serial.print(" deg/s\t\t");
    Serial.print("Y-gyro rate: "); Serial.print(capteur->gyro[1], 4); Serial.print(" deg/s\t\t");
    Serial.print("Z-gyro rate: "); Serial.print(capteur->gyro[2], 4); Serial.println(" deg/s");
    */
    Serial.print("X-mag field: "); Serial.print(capteur->mag[0], 4); Serial.print(" mG\t\t");
    Serial.print("Y-mag field: "); Serial.print(capteur->mag[1], 4); Serial.print(" mG\t\t");
    Serial.print("Z-mag field: "); Serial.print(capteur->mag[2], 4); Serial.println(" mG"); 
  #endif

}

/* --------------------------------------------------------------- */
/* ----------------- Mise a jour de l'orientation ---------------- */
/* --------- des donnees de l'accelerometre et gyroscope --------- */
/* --------------------------------------------------------------- */
void getOrientationWithoutMag
(
  byte address,    /* Adresse i2c du capteur */
  t_IMU * capteur       /* Reference sur le capteur */
)
{
  /* Variables locales */
  int16_t rawData[3] = {0, 0, 0};
  int i;

  /* Recuperation et traitement des donnees de l'accelerometre */
  readAccelData(rawData, address);
  for (i = 0; i < 3; i++)     capteur->acc[i] = (float)rawData[i] * ACC_RESOLUTION - capteur->accBias[i];

  /* Recuperation et traitement des donnees du gyroscope */
  readGyroData(rawData, address);
  for (i = 0; i < 3; i++)     capteur->gyro[i] = (float)rawData[i] * GYRO_RESOLUTION - capteur->gyroBias[i];
}

/* --------------------------------------------------------------- */
/* ------ Initialisation de l'accelerometre et du gyroscope ------ */
/* --------------------------------------------------------------- */
void initMPU9250
(
  byte address    /* Adresse i2c du capteur */
)
{
  /* Reset de l'offset du gyroscope */
  writeByte(address, XA_OFFSET_H, 0x00);
  writeByte(address, XA_OFFSET_L, 0x00);
  writeByte(address, YA_OFFSET_H, 0x00);
  writeByte(address, YA_OFFSET_L, 0x00);
  writeByte(address, ZA_OFFSET_H, 0x00);
  writeByte(address, ZA_OFFSET_L, 0x00);

  /* Reset de l'offset de l'accelerometre */
  writeByte(address, XG_OFFS_USRL, 0x00);
  writeByte(address, XG_OFFS_USRH, 0x00);
  writeByte(address, YG_OFFS_USRL, 0x00);
  writeByte(address, YG_OFFS_USRH, 0x00);
  writeByte(address, ZG_OFFS_USRL, 0x00);
  writeByte(address, ZG_OFFS_USRH, 0x00);

  /* Reset du capteur et utilisation de l'oscillateur interne */
  writeByte(address, PWR_MGMT_1, 0x80);
  /* Activation de tous les capteurs */
  writeByte(address, PWR_MGMT_2, 0x00);

  /* Attente de l'activation des capteurs */
  delay(200);

  /* Configuration du filtre passe bas du gyroscope */
  /* Bande passante du gyroscope à 41 Hz, delais de 5,9 ms */
  /* Frequence d'echantillonage de 1 KHz. FSYNC desactive */
  writeByte(address, CONFIG, 0x03);

  /* Echantillonage = sortie du gyroscope / (1 + SMPLRT_DIV) : ici = 200 Hz */
  writeByte(address, SMPLRT_DIV, 0x04);  

  /* Configuration de l'echelle du gyroscope a 250 dps */
  writeByte(address, GYRO_CONFIG, 0x00);
  
  /* Configuration de l'echelle de l'accelerometre a 2g */
  writeByte(address, ACCEL_CONFIG, 0x00);

  /* Configuration du filtre passe bas de l'accelerometere */
  /* Bande passante du gyroscope à 41 Hz, delais de 11,80 ms */
  /* Frequence d'echantillonage de 1 KHz */
  writeByte(address, ACCEL_CONFIG2, 0x03);
}

/* --------------------------------------------------------------- */
/* --------------- Initilisation du magnetometre ----------------- */
/* --------------------------------------------------------------- */
void initAK8963
(
  byte address,         /* Adresse i2c du capteur */
  t_IMU * capteur       /* Référence sur le capteur */
)
{
  writeByte(address, USER_CTRL, 0x20);                /* Mode i2c Master */
  delay(100);
  writeByte(address, I2C_MST_CTRL, 0x0D);             /* Configuration de l'i2c en multi-maitre a 400KHz */
  delay(100);
  writeByte(address, I2C_SLV0_ADDR, AK8963_ADDRESS);  /* Renseigne l'adresse i2c du peripherique esclave 0 */
  delay(100);
  writeByte(address, I2C_SLV0_REG, AK8963_CNTL2);     /* Indique l'adresse de l'esclave i2c 0 a partir de laquelle effectuer l'ecriture */
  delay(100);
  writeByte(address, I2C_SLV0_DO, 0x01);              /* Reset du magnetometre */
  delay(100);
  writeByte(address, I2C_SLV0_CTRL, 0x81);            /* Activation de l'i2c et ecriture d'1 octet */
  delay(100);
  writeByte(address, I2C_SLV0_REG, AK8963_CNTL1);     /* Indique l'adresse de l'esclave i2c 0 a partir de laquelle effectuer l'ecriture */
  delay(100);
  writeByte(address, I2C_SLV0_DO, 0x16);              /* Register value to continuous measurement 100 Hz in 16 bit */
  delay(100);
  writeByte(address, I2C_SLV0_CTRL, 0x81);            /* Activation de l'i2c et ecriture d'1 octet */
  delay(100);

  /* Configuration des valeurs de sensibilite du magnetometre (valeurs usine, acces en lecture seule uniquement) */
  capteur->magSensitivity[0] = AK8963_SENS_ASAX;
  capteur->magSensitivity[1] = AK8963_SENS_ASAY;
  capteur->magSensitivity[2] = AK8963_SENS_ASAZ;
}

/* --------------------------------------------------------------- */
/* -------- Lecture des données brutes de l'ACCELEROMETRE -------- */
/* --------------------------------------------------------------- */
void readAccelData
(
  int16_t * destination,    /* Donnees brutes de l'accelerometre */
  byte address              /* Adresse i2c du capteur */
)
{
  /* Variable locale */
  uint8_t rawData[6]; /* Permet de stocker les données sur X, Y et Z */

  /* Lecture des données (sur 6 octets de 8 bit) */
  readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

/* --------------------------------------------------------------- */
/* ----------- Lecture des données brutes du GYROSCOPE ----------- */
/* --------------------------------------------------------------- */
void readGyroData
(
  int16_t * destination,    /* Donnees brutes du gyroscope */
  byte address              /* Adresse i2c du capteur */
)
{
  /* Variable locale */
  uint8_t rawData[6]; /* Permet de stocker les données sur X, Y et Z */

  /* Lecture des données (sur 6 octets de 8 bit) */
  readBytes(address, GYRO_XOUT_H, 6, &rawData[0]);
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

/* --------------------------------------------------------------- */
/* ---------- Lecture des données brutes du MAGNETOMETRE --------- */
/* --------------------------------------------------------------- */
void readMagData
(
  int16_t * destination,    /* Donnes brutes du magnetometre */
  byte address              /* Adresse i2c du magnetometre */
)
{
  /* Variable locale */
  uint8_t rawData[6];
  int i;

  /* Renseigne l'adresse i2c du peripherique esclave 0 et indique une lecture */
  writeByte(address, I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG);
  /* Indique l'adresse de l'esclave i2c 0 a partir de laquelle effectuer la lecture */
  writeByte(address, I2C_SLV0_REG, AK8963_XOUT_L);
  /* Lecture de 6 bit depuis le magnetometre */
  writeByte(address, I2C_SLV0_CTRL, 0x87);

  readBytes(address, EXT_SENS_DATA_00, 7, &rawData[0]);
  for (i = 0; i < 3; i++)
  {
      destination[i] = ((int16_t)rawData[i*2+1] << 8) | rawData[i*2];
  }
}

/* --------------------------------------------------------------- */
/* --------- Affichage des offsets et echelles du capteur -------- */
/* --------------------------------------------------------------- */
void printSensorValues
(
  t_IMU capteurCam
)
{
  Serial.print("magCamBiasX : ");
  Serial.print(capteurCam.magBias[0]);
  Serial.print(" magCamBiasY : ");
  Serial.print(capteurCam.magBias[1]);
  Serial.print(" magCamBiasZ : ");
  Serial.print(capteurCam.magBias[2]);
  Serial.print(" magCamScaleX : ");
  Serial.print(capteurCam.magScale[0]);
  Serial.print(" magCamScaleY : ");
  Serial.print(capteurCam.magScale[1]);
  Serial.print(" magCamScaleZ : ");
  Serial.println(capteurCam.magScale[2]);

  Serial.print("gyroCamBiasX : ");
  Serial.print(capteurCam.gyroBias[0], 5);
  Serial.print(" - gyroCamBiasY : ");
  Serial.print(capteurCam.gyroBias[1], 5);
  Serial.print(" - gyroCamBiasZ : ");
  Serial.print(capteurCam.gyroBias[2], 5);
  Serial.print(" - accCamBiasX : ");
  Serial.print(capteurCam.accBias[0], 5);
  Serial.print(" - accCamBiasY : ");
  Serial.print(capteurCam.accBias[1], 5);
  Serial.print(" - accCamBiasZ : ");
  Serial.println(capteurCam.accBias[2], 5);
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
    dest[i++] = Wire.read();         // Put read results in the Rx buffer
  }
}
