const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 5;

// SPWM (Sine Wave)
const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };

int currentStepA;
int currentStepB;
int currentStepC;
int sineArraySize;
int increment = 0;
boolean direct = 1; // direction true=forward, false=backward
 
//////////////////////////////////////////////////////////////////////////////
 
void setup() {
 
  analogWriteFrequency(IN1, 32000);
  analogWriteFrequency(IN2, 32000);
  analogWriteFrequency(IN3, 32000);
 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
 
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number
}
 
//////////////////////////////////////////////////////////////////////////////
 
void loop() {
 
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
 
  if(currentStepC > sineArraySize)  currentStepC = 0;
  if(currentStepC < 0) currentStepC = sineArraySize; 
  
  /// Control speed by this delay
  delay(10);
 
}
