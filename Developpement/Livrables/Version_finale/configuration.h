
/* Adresse des capteurs sur le bus i2c */
#define MPU9250_CAM_ADDRESS   0x68
#define AK8963_ADDRESS        0x0C

/* Definition des numeros de sortie des moteurs sur le microcontroleur */
#define YAW_MOTOR_P1      9
#define YAW_MOTOR_P2      6
#define YAW_MOTOR_P3      10
#define YAW_MOTOR_EN      8

#define PITCH_MOTOR_P1    20
#define PITCH_MOTOR_P2    21
#define PITCH_MOTOR_P3    22
#define PITCH_MOTOR_EN    15

#define ROLL_MOTOR_P1     3
#define ROLL_MOTOR_P2     4
#define ROLL_MOTOR_P3     5
#define ROLL_MOTOR_EN     7

/* Sens de rotation des moteurs */
#define SENS_MOTOR_YAW          -1
#define SENS_MOTOR_PITCH        -1
#define SENS_MOTOR_ROLL         -1

/* Constantes liees au capteur MPU-9250 */
#define ACC_RESOLUTION   (2.0 / 32768.0)          /* Resolution de l'accelerometre */
#define GYRO_RESOLUTION  (250.0 / 32768.0)        /* Resolution du gyroscope */
#define MAG_RESOLUTION   1.5    /* Resolution du magnetometre */

/* Valeurs de sensibilité du magnétomètre (ASAX, ASAY, ASAZ) */
#define AK8963_SENS_ASAX  1.11
#define AK8963_SENS_ASAY  1.07
#define AK8963_SENS_ASAZ  1.50

/* Adresses memoire EEPROM pour le stockage des donnees de calibration */
#define ADDR_GYRO_CAM_BIAS      0   /* 3 float */
#define ADDR_ACC_CAM_BIAS       12  /* 3 float */
#define ADD_MAG_CAM_BIAS        24  /* 3 float */
#define ADD_MAG_CAM_SCALE       36  /* 3 float */
#define FIRST_CALIBRATION_FLAG  60  /* 1 int */

#define ADDR_KP_PARAMETERS      70  /* 3 float : Yaw, Pitch, Roll */
#define ADDR_KI_PARAMETERS      82  /* 3 float : Yaw, Pitch, Roll */
#define ADDR_KD_PARAMETERS      94  /* 3 float : Yaw, Pitch, Roll */
#define ADDR_INTERRUPT_VALUES   106 /* 3 int */
