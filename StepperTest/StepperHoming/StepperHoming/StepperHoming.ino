//Written By Nikodem Bartnik - nikodembartnik.pl
#define STEPPER_PIN_1 2
#define STEPPER_PIN_2 3
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5

#define STEPPER_IR_SENSOR_PIN 6

bool direction = false; //False == CW Motion, True = CCW Motion
int step_number = 0;

float angle = 0; //Clock-wise = Positive angle gain, Counter-clockwise = Negative angle gain
float angleStepRatio = 5.625/64; //5.625 degrees/64 steps approximately  == 0.088 degrees/step
float gearStepperRatio = 1;
int stepperHomingExtraSteps = 36;

void setup() {
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  pinMode(STEPPER_IR_SENSOR_PIN, INPUT);
  stepperHoming(); //Initialize Angle 0;
}

void loop() {
 
  //OneStep(direction);
  //Serial.println(angle);
  delay(2);
}

void stepperHoming()
{
  while(digitalRead(STEPPER_IR_SENSOR_PIN) == LOW)
  {
    OneStep(direction);
    delay(2);
  }
  while(stepperHomingExtraSteps > 0)
  {
    OneStep(direction);
    delay(2);
    stepperHomingExtraSteps--;
  }
  angle = 0;
}

void OneStep(bool dir){
  if(dir){
    angle -= angleStepRatio;
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    } 
  }
  else{
    angle += angleStepRatio;
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW); 
        break; 
    } 
  }
  step_number++;
  if(step_number > 3){
    step_number = 0;
  }

  if(angle > 360.00)
  {
    angle = angle - 360.00;
  }
  else if(angle < 0)
  {
    angle = 360.00 + angle;
  }
}