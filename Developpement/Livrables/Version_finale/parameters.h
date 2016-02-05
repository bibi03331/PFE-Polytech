
/* Definition des coefficient pour le PID de chaque moteur */
#define KP_Y  0.001
#define KI_Y  0.003
#define KD_Y  1.0

#define KP_P  0.8 
#define KI_P  0.008
#define KD_P  1000.0

#define KP_R  0.001
#define KI_R  0.005
#define KD_R  1.0

/* Frequence des signaux PWM pour la commande des moteurs brushless en Hz */
#define PWM_FREQUENCY           18000

/* Frequence de boucle d'asservissement en Hz */
#define PID_FREQUENCY           50000

/* Frequence de rafraichissement des données du capteur en Hz */
#define UPDATE_CPT_FREQUENCY    200

/* Frequence de boucle de l'algorithme de Madgwick en Hz */
#define MADGWICK_FREQUENCY      1000

/* Moteurs brushless avec 7 trinomes d'enroulement electrique */
#define NB_ELEC_BLOCS           7

#define ENABLE_ROLL_AXIS    ((const char *)"en_roll_axis")
#define ENABLE_PITCH_AXIS   ((const char *)"en_pitch_axis")
#define ENABLE_YAW_AXIS     ((const char *)"en_yaw_axis")
#define KP_ROLL             ((const char *)"Kp_Roll")
#define KI_ROLL             ((const char *)"Ki_Roll")
#define KD_ROLL             ((const char *)"Kd_Roll")
#define KP_PITCH            ((const char *)"Kp_Pitch")
#define KI_PITCH            ((const char *)"Ki_Pitch")
#define KD_PITCH            ((const char *)"Kd_Pitch")
#define KP_YAW              ((const char *)"Kp_Yaw")
#define KI_YAW              ((const char *)"Ki_Yaw")
#define KD_YAW              ((const char *)"Kd_Yaw")
#define FREQ_CPT            ((const char *)"freq_cpt")
#define FREQ_MOTORS         ((const char *)"freq_motors")
#define FREQ_PID            ((const char *)"freq_PID")
#define FREQ_MADGWICK       ((const char *)"freq_madgwick")
#define RECORD_PARAMETERS   ((const char *)"record_parameters")
