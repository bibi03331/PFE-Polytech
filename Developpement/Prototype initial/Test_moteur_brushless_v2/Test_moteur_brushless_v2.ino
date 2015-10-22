const int motorDelayActual = 20;
const int motorPin1 =9;
const int motorPin2 =10;
const int motorPin3 =11;
const int motorPinState[]={127,111,94,78,64,50,37,26,17,9,4,1,0,1,4,9,17,26,37,50,64,78,
                           94,111,127,144,160,176,191,205,218,229,238,245,251,254,255,
                           254,251,245,238,229,218,205,191,176,160,144,127};
int currentStepA = 0;
int currentStepB = 16;
int currentStepC = 32;
long lastMotorDelayTime = 0;

void setup () {
 
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  
  setPwmFrequency(motorPin1);
  setPwmFrequency(motorPin2);
  setPwmFrequency(motorPin3);
  
}
void loop () {
  
  if((millis() - lastMotorDelayTime) > motorDelayActual) {
    
      currentStepA = currentStepA ++;
      if(currentStepA > 47) currentStepA = 0;
      if(currentStepA < 0) currentStepA = 47;
      
      currentStepB = currentStepB ++;
      if(currentStepB > 47) currentStepB = 0;
      if(currentStepB < 0) currentStepB = 47;
      
      currentStepC = currentStepC ++;
      if(currentStepC > 47) currentStepC = 0;
      if(currentStepC < 0) currentStepC = 47;
      
      lastMotorDelayTime =millis();
      analogWrite(motorPin1, motorPinState[currentStepA]);
      analogWrite(motorPin2, motorPinState[currentStepB]);
      analogWrite(motorPin3, motorPinState[currentStepC]);
  
  }

}

void setPwmFrequency(int pin) {
  
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
  
}
