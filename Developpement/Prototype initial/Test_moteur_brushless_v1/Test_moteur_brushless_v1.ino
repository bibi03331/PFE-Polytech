
const int motorDelayActual = 150;
const int motorPin1 =9;
const int motorPin2 =10;
const int motorPin3 =11;
const int motorPinState[]={1,1,1,0,0,0};
int currentStepA=0;
int currentStepB=2;
int currentStepC=4;
long lastMotorDelayTime = 0;

void setup () {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(5, OUTPUT); 
 
 
  digitalWrite(5, HIGH);
}

void loop () {
  
  if((millis() - lastMotorDelayTime) > motorDelayActual){
    
      currentStepA = currentStepA ++;
      if(currentStepA > 5) currentStepA = 0;
      if(currentStepA < 0) currentStepA = 5;
      
      currentStepB = currentStepB ++;
      if(currentStepB > 5) currentStepB = 0;
      if(currentStepB < 0) currentStepB = 5;
      
      currentStepC = currentStepC ++;
      if(currentStepC > 5) currentStepC = 0;
      if(currentStepC < 0) currentStepC = 5;
      
      lastMotorDelayTime =millis();
      analogWrite(motorPin1, 254 * motorPinState[currentStepA]);
      analogWrite(motorPin2, 254 * motorPinState[currentStepB]);
      analogWrite(motorPin3, 254 * motorPinState[currentStepC]);
  
  }
}

