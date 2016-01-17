
#define F_CPU 72000000
#define F_TIMER (F_CPU/2)

#define PWM_FREQ 32000  // 32 Khz
#define CMD_FREQ 50     // 50 Hz

#define ROLL_MOTOR_P1 3
#define ROLL_MOTOR_P2 4
#define ROLL_MOTOR_P3 5
#define ROLL_MOTOR_EN 7

#define PITCH_MOTOR_P1 6
#define PITCH_MOTOR_P2 9
#define PITCH_MOTOR_P3 10
#define PITCH_MOTOR_EN 8

#define YAW_MOTOR_P1 20
#define YAW_MOTOR_P2 21
#define YAW_MOTOR_P3 22
#define YAW_MOTOR_EN 15

#define PORT_PCR_ROLL_MOTOR_P1 PORTA_PCR12
#define PORT_PCR_ROLL_MOTOR_P2 PORTA_PCR13
#define PORT_PCR_ROLL_MOTOR_P3 PORTD_PCR7
#define PORT_PCR_ROLL_MOTOR_EN PORTD_PCR2

#define PORT_PCR_PITCH_MOTOR_P1 PORTD_PCR4
#define PORT_PCR_PITCH_MOTOR_P2 PORTC_PCR3
#define PORT_PCR_PITCH_MOTOR_P3 PORTC_PCR4
#define PORT_PCR_PITCH_MOTOR_EN PORTD_PCR3

#define PORT_PCR_YAW_MOTOR_P1 PORTD_PCR5
#define PORT_PCR_YAW_MOTOR_P2 PORTD_PCR6
#define PORT_PCR_YAW_MOTOR_P3 PORTC_PCR1
#define PORT_PCR_YAW_MOTOR_EN PORTC_PCR6

#define FTM_ROLL_MOTOR_P1 FTM1_C0V
#define FTM_ROLL_MOTOR_P2 FTM1_C1V
#define FTM_ROLL_MOTOR_P3 FTM0_C7V

#define FTM_PITCH_MOTOR_P1 FTM0_C4V
#define FTM_PITCH_MOTOR_P2 FTM0_C2V
#define FTM_PITCH_MOTOR_P3 FTM0_C3V

#define FTM_YAW_MOTOR_P1 FTM0_C5V
#define FTM_YAW_MOTOR_P2 FTM0_C6V
#define FTM_YAW_MOTOR_P3 FTM0_C0V

#define MIN_CFG_MOTOR 100
#define MAX_CFG_MOTOR 6000

// SPWM (Sine Wave)
const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
//const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

int currentStepA, currentStepB, currentStepC;
int sineArraySize;
int increment = 0;
boolean direct = 1; // Direction true=forward, false=backward

uint16_t yawMotorSpeed = 1000;
uint16_t cmdRollMotorCfg = (((MAX_CFG_MOTOR - MIN_CFG_MOTOR) / 1024) * yawMotorSpeed) + MIN_CFG_MOTOR;

// POUBELLE
int nbTable = 0;
int consigne = 0;
int img = 0;

//////////////////////////////////////////////////////////////////////////////
 
void setup() {

  configPwmFrequency();
  configInputsOutputs();
  configInterrupts();
  enableMotors();
  
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number
  
}
 
//////////////////////////////////////////////////////////////////////////////
 
void loop() {

  if (Serial.available() > 0) {
    consigne = Serial.parseInt();
    Serial.print("Consigne : ");
    Serial.println(consigne);
    consigne *= 1.4; // (6 electrical steps x 72 + 1 algorithmique step x 72) / 360
  }
 
}

void pit0_isr(void) {

  setPwm(ROLL_MOTOR_P1, pwmSin[currentStepA]);
  setPwm(ROLL_MOTOR_P2, pwmSin[currentStepB]);
  setPwm(ROLL_MOTOR_P3, pwmSin[currentStepC]);

  if (consigne >= 0 && consigne <= 360) {
    
    if (consigne > img) increment = 1;
    else if (consigne < img) increment = -1;
    else increment = 0;
  }
  
  currentStepA = currentStepA + increment;
  currentStepB = currentStepB + increment;
  currentStepC = currentStepC + increment;
  img = img + increment;
 
  //Check for lookup table overflow and return to opposite end if necessary
  if(currentStepA > sineArraySize) currentStepA = 0;
  if(currentStepA < 0) currentStepA = sineArraySize;
  if(currentStepB > sineArraySize) currentStepB = 0;
  if(currentStepB < 0) currentStepB = sineArraySize;
  if(currentStepC > sineArraySize) currentStepC = 0;
  if(currentStepC < 0) currentStepC = sineArraySize;
  
  PIT_TFLG0 = 1; 
}

void pit1_isr(void) {

  
  PIT_TFLG1 = 1; 
}

void setPwm(uint8_t id, uint8_t pwmVal) {

  uint32_t pwm;

  if (id == 3 || id == 4) {
    pwm = ((uint32_t)pwmVal * (uint32_t)(FTM1_MOD + 1)) >> 8;
  }
  else {
    pwm = ((uint32_t)pwmVal * (uint32_t)(FTM0_MOD + 1)) >> 8;
  }

  switch (id) {
    case ROLL_MOTOR_P1:
      FTM_ROLL_MOTOR_P1 = pwm;
      PORT_PCR_ROLL_MOTOR_P1 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case ROLL_MOTOR_P2:
      FTM_ROLL_MOTOR_P2 = pwm;
      PORT_PCR_ROLL_MOTOR_P2 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case ROLL_MOTOR_P3:
      FTM_ROLL_MOTOR_P3 = pwm;
      PORT_PCR_ROLL_MOTOR_P3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
      
    case PITCH_MOTOR_P1:
      FTM_PITCH_MOTOR_P1 = pwm;
      PORT_PCR_PITCH_MOTOR_P1 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case PITCH_MOTOR_P2:
      FTM_PITCH_MOTOR_P2 = pwm;
      PORT_PCR_PITCH_MOTOR_P2 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case PITCH_MOTOR_P3:
      FTM_PITCH_MOTOR_P3 = pwm;
      PORT_PCR_PITCH_MOTOR_P3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;

    case YAW_MOTOR_P1:
      FTM_YAW_MOTOR_P1 = pwm;
      PORT_PCR_YAW_MOTOR_P1 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case YAW_MOTOR_P2:
      FTM_YAW_MOTOR_P2 = pwm;
      PORT_PCR_YAW_MOTOR_P2 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
    case YAW_MOTOR_P3:
      FTM_YAW_MOTOR_P3 = pwm;
      PORT_PCR_YAW_MOTOR_P3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
      break;
  }
}

void configInputsOutputs(void) {

  PORT_PCR_ROLL_MOTOR_P1 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_ROLL_MOTOR_P2 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_ROLL_MOTOR_P3 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_ROLL_MOTOR_EN = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  
  PORT_PCR_PITCH_MOTOR_P1 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_PITCH_MOTOR_P2 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_PITCH_MOTOR_P3 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_PITCH_MOTOR_EN = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  
  PORT_PCR_YAW_MOTOR_P1 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_YAW_MOTOR_P2 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_YAW_MOTOR_P3 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  PORT_PCR_YAW_MOTOR_EN = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
}

void configPwmFrequency(void) {

  uint8_t prescale, minfreq;

  for (prescale = 0; prescale < 7; prescale++) {
    minfreq = (float)(F_TIMER >> prescale) / 65536.0f;
    if (PWM_FREQ >= minfreq) break;
  }

  FTM0_SC = 0;
  FTM0_CNT = 0;
  FTM0_MOD = (float)((F_TIMER/2) >> prescale) / PWM_FREQ - 0.5f;
  FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(prescale);

  FTM1_SC = 0;
  FTM1_CNT = 0;
  FTM1_MOD = (float)((F_TIMER/2) >> prescale) / PWM_FREQ - 0.5f;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(prescale);
  
}

void configInterrupts(void) {

  SIM_SCGC6 |= SIM_SCGC6_PIT; // Activate the used by clock PIT
  PIT_MCR = 0x00;             // Active PIT timers

  /* Interrupt 1 : Used for control speed motors */
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  PIT_LDVAL0 = F_CPU / cmdRollMotorCfg;
  PIT_TCTRL0 = 0x02;
  PIT_TCTRL0 |= 0x01;
  PIT_TFLG0 |= 1;

  /* Intrrupt 2 : Update Gyro and Acc values */
  /*
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  PIT_LDVAL1 = 18000000;
  PIT_TCTRL1 = 0x02;
  PIT_TCTRL1 |= 0x01;
  PIT_TFLG1 |= 1;
  */
}

void enableMotors(void) {

  PORT_PCR_ROLL_MOTOR_EN |= (PORT_PCR_PE | PORT_PCR_PS);
  PORT_PCR_PITCH_MOTOR_EN |= (PORT_PCR_PE | PORT_PCR_PS);
  PORT_PCR_YAW_MOTOR_EN |= (PORT_PCR_PE | PORT_PCR_PS);
}


