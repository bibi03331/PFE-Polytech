/* Structure pour l'asservissement PID */
typedef struct
{
  float consigne;           /* Consigne de commande du moteur */
  float erreur;             /* Erreur : difference entre la consigne et la mesure */
  float sommeErreurs;       /* Sommes des erreur : integral */
  float variationErreur;    /* Variation le l'erreur dans le temps */
  float erreurPrecedente;   /* Erreur precedente */
  float kp;                 /* Coefficient proportionnel */
  float ki;                 /* Coefficient intégral */
  float kd;                 /* Coefficient derive */
} t_PID;

/* Structure pour la commande des moteurs */
typedef struct
{
  int consigne;         /* Consigne de commande du moteur */
  int image;            /* Image de la position du moteur : de 0 a 360 degres */
  int input[3];         /* Numeros de broche du microcontroleur pour chaque signal PWM du moteur */
  int enableMotor;      /* Numero de broche du microcontroleur pour le signal TOR d'activation du moteur */
  int currentStep[3];   /* Valeur courante de la lookup table pour chaque signal PWM du moteur */
  int increment;        /* Increment pour la position dans la lookup table */
  int sensRotation;     /* Sens de rotation du moteur : Permet d'inverser le sens de rotation en fonction du montage */
} t_motor;

typedef struct
{
  float magScale[3];                                /* Echelle pour X, Y, Z */
  float magSensitivity[3];                          /* Sensibilite du magnetometre pour X, Y, Z */
  float accBias[3];                                 /* Offset de l'accelerometre pour X, Y, Z */
  float gyroBias[3];                                /* Offset du gyroscope pour X, Y, Z */
  float magMax[3] = {0.0f, 0.0f, 0.0f};             /* Valeurs max du magnetometre pour X, Y, Z (Etalonnage) */
  float magMin[3] = {0.0f, 0.0f, 0.0f};             /* Valeurs min du magnetometre pour X, Y, Z (Etalonnage) */
  float magBias[3] = {0.0f, 0.0f, 0.0f};            /* Offset du magnetometre pour X, Y, Z */
  float acc[3];                                     /* Valeurs brutes de l'accelerometre pour X, Y, Z */
  float gyro[3];                                    /* Valeurs brutes du gyroscope pour X, Y, Z */
  float mag[3];                                     /* Valeurs brutes du magnetometre pour X, Y, Z */
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            /* Quaternion issu du filtre de Madgwick */
  float ypr[3];                                     /* Orientation sur Yaw Pitch Roll */
  
} t_IMU;
