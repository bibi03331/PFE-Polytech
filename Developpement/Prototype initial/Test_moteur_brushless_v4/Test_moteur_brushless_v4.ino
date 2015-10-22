//
// Slow and precise BLDC motor driver using SPWM and SVPWM modulation
// Part of code used from http://elabz.com/
// (c) 2015 Ignas Gramba www.berryjam.eu
//
 
 
const int EN = 5;
 
const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;

const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
 
int currentStepA, currentStepB, currentStepC;
int sineArraySize;
int increment = 0;
boolean direct = 1; // direction true=forward, false=backward

int valOCR0A = 0;
int cmpt = 1;
 
//////////////////////////////////////////////////////////////////////////////
 
void setup() {
 
  setPwmFrequency(IN1); // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);
 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  
  pinMode(EN, OUTPUT); 
  digitalWrite(EN, HIGH);
 
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number
  
  configureTMR0();
  configureTMR3();
  sei();
}
 
//////////////////////////////////////////////////////////////////////////////
 
void loop() {
 
}

ISR(TIMER0_COMPA_vect){
  updateMotor();
}

ISR(TIMER3_COMPA_vect){
  if(cmpt >= 5) {
    if (valOCR0A < (249 - 12)) valOCR0A += 12;
    else valOCR0A = 0;
    cmpt = 0;
  }
  else cmpt++;
  
}

void updateMotor() {
  analogWrite(IN1, pwmSin[currentStepA]);
  analogWrite(IN2, pwmSin[currentStepB]);
  analogWrite(IN3, pwmSin[currentStepC]);  
  
  if (direct==true) increment = 1;
  else increment = -1;     
 
  currentStepA = currentStepA + increment;
  currentStepB = currentStepB + increment;
  currentStepC = currentStepC + increment;
 
  //Check for lookup table overflow and return to opposite end if necessary
  if(currentStepA > sineArraySize)  currentStepA = 0;
  if(currentStepA < 0)  currentStepA = sineArraySize;
 
  if(currentStepB > sineArraySize)  currentStepB = 0;
  if(currentStepB < 0)  currentStepB = sineArraySize;
 
  if(currentStepC > sineArraySize) currentStepC = 0;
  if(currentStepC < 0) currentStepC = sineArraySize;
  
  OCR0A = valOCR0A;
}

void configureTMR0() {
  
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  
  // set compare match register for x khz increments
  OCR0A = valOCR0A;// = (16 * 10^6) / (x * 64) - 1 (must be < 256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // prescaler = 1024
  TCCR0B = TCCR0B & 0b11111000 | 5;  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);  
}

void configureTMR3() {
  
  TCCR3A = 0;// set entire TCCR2A register to 0
  TCCR3B = 0;// same for TCCR2B
  TCNT3  = 0;//initialize counter value to 0
  
  // set compare match register for x khz increments
  OCR3A = 0;// = (16 * 10^6) / (x * 64) - 1 (must be < 256)
  // turn on CTC mode
  TCCR3A |= (1 << WGM31);
  // prescaler = 0
  TCCR3B = TCCR3B & 0b11111000 | 5;  
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);  
}
 
void setPwmFrequency(int pin) {
  if(pin == 9 || pin == 10) {
    TCCR2B = TCCR0B & 0b11111000 | 1;
  }
  else if(pin == 11) {
    TCCR1B = TCCR2B & 0b11111000 | 1;
  }
}
