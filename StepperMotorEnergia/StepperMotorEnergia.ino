
int LED = RED_LED;
int PUSHBUTTON = 5;
int STEPPORT = 9;
int dirPin = 10;
int MS1PORT = 11;
int MS2PORT = 12;

const float motorAngle = 1.8;
const float stepSize = 1;//full=1, half=0.5, quarter=0.25, etc...
int numOfSteps = 0;
int directionOfMotor = 0;
int transmit = 0;

void stepperRotate(float rotation, float rpm);

void setup() {
  Serial.begin(9600);
  
  pinMode(PUSHBUTTON, INPUT_PULLUP);
  pinMode(STEPPORT, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(MS1PORT, OUTPUT);
  pinMode(MS2PORT, OUTPUT);

  digitalWrite(STEPPORT, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(MS1PORT, LOW);
  digitalWrite(MS2PORT, LOW);

 
}

void loop() {

  if(Serial.available() > 0) 
  { 
    numOfSteps = Serial.read();
    delay(1);
    directionOfMotor = Serial.read();

    Serial.println(numOfSteps);
    Serial.println(directionOfMotor);
  
    stepperRotate(directionOfMotor, 100);
    }
}

void stepperRotate(float rotation, float rpm) {
  if (rotation == 0) {
    digitalWrite(dirPin, HIGH);
  }
  else if(rotation == 1) {
    digitalWrite(dirPin, LOW);
  }

  float stepsPerRotation = (360.00 / motorAngle) / stepSize;

  float totalSteps = rotation * stepsPerRotation;

  unsigned long stepPeriodmicroSec = ((60.0000 / (rpm * stepsPerRotation)) * 1E6 / 2.0000) - 5;


  for (unsigned long i = 0; i < numOfSteps; i++) {
    digitalWrite(STEPPORT, HIGH);
    delayMicroseconds(stepPeriodmicroSec);
    digitalWrite(STEPPORT, LOW);
    delayMicroseconds(stepPeriodmicroSec);
  }
}
