#include <Servo.h> 

Servo torch;  
 
// P I N S  
int ServoPin      = 5;
int CutterCamPin  = 6;
int RodStopperPin = 7;
int RodPusherPin  = 8;
int SpinMotorPin  = 9;
int CutSwitchPin  = 10;

// C O N S T A N T S
int torchRestingAngle = 0;
int torchBurningAngle = 20;

int torchMeltBallDuration = 3500;
int torchSoftenBendDuration = 1000;

int advanceRodToMeltingDuration = 500;
int advanceRodToBendDuration = 500;

int postOperation10msDelay = 10;
int postOperationShortDelay = 50;
int postOperationMediumDelay = 200;
int postOperationLongDelay = 500;
int postOperation1SecDelay = 1000;

int postOperationWhileDelay = 10;

int loopCounter = 0;

/***********************************
 *  S E T U P                      *
 ***********************************/
void setup() {
  Serial.println("Nose Screw workflow setup.");
  Serial.begin(9600); 

  torch.attach(ServoPin);

  pinMode(SpinMotorPin,  OUTPUT);       
  pinMode(RodPusherPin,  OUTPUT);
  pinMode(RodStopperPin, OUTPUT);
  pinMode(CutterCamPin,  OUTPUT);
  pinMode(CutSwitchPin,  INPUT_PULLUP);
  //pinMode(CutSwitchPin,  INPUT);
  
  if (cutSwitchIsOff()) {
    runCutterUntilSwitchEngaged();
  }
}
 
/***********************************
 *  W O R K F L O W  M A I N       *
 ***********************************/ 
void loop() {
  Serial.println("Begin nose screw workflow.");
  
  initializeAll();

  advanceRodToFirstPosition();

  meltBallUsingTorch();

  advanceRodToFinalPosition();

  bendRodUsingTorch();

  //cutRodUsingDuration(3300,100);
  cutRodUsingSwitchCondition();
    
  Serial.println("End nose screw workflow.");  
}

void initializeAll() {

  loopCounter++;
  Serial.println(loopCounter);
//  stopUsingPin(SpinMotorPin);
//  stopUsingPin(RodPusherPin);
//  stopUsingPin(RodStopperPin);
  //stopUsingPin(CutterCamPin);
//  if (cutSwitchIsOff()) {
//    runCutterUntilSwitchEngaged();
//  }

  //setTorchAndDelay(torchRestingAngle,postOperationLongDelay); 
}

/************************************
 *  W O R K F L O W  M E T H O D S  *
 ************************************/ 
void advanceRodToFirstPosition() {
  Serial.println("- Advance Rod to First Position");
  engageBallStop(postOperationMediumDelay);
  advanceRod(advanceRodToMeltingDuration,postOperationMediumDelay);
  retractBallStop(postOperationShortDelay);
}

void advanceRodToFinalPosition() {
  Serial.println("- Advance Rod to Final Position");
  advanceRod(advanceRodToBendDuration,postOperationShortDelay);
}

void meltBallUsingTorch() {
  Serial.println("- Start Melt Ball Sequence.");
  spinRodMotor(); 
  operateTorch(torchBurningAngle,torchMeltBallDuration,600);
  stopRodMotor(postOperationShortDelay);
}

void bendRodUsingTorch() {
  Serial.println("- Start Bend Sequence.");
  operateTorch(torchBurningAngle,torchSoftenBendDuration,postOperationLongDelay);
}

void cutRodUsingDuration(int cutCamDuration, int finalDelay) {
  Serial.println("- Cut Rod using Duration.");
  startWithDelay(CutterCamPin,cutCamDuration);
  stopWithDelay(CutterCamPin,finalDelay);
}

void cutRodUsingSwitchCondition() {
  Serial.println("- Cut Rod using Switch.");
  runCutterCamUntilSwitchIsOn();
}

/************************************
 *  C U T T E R      M E T H O D S  *
 ************************************/ 
void runCutterCamUntilSwitchIsOn() {
  Serial.println(" -  Start Cutter Cam");
  if (cutSwitchIsOff()) {
    Serial.println("  -  Starting in the OFF position");
    runCutterUntilSwitchEngaged();
  } else {
    Serial.println("  -  Starting in the ON position");
    runCutterUntilSwitchDisengaged();
    Serial.println("  -  Switch is now OFF, wait for it to come back on");
    runCutterUntilSwitchEngaged();
  }
  Serial.println(" -  End Cutter Cam");
}

void runCutterUntilSwitchEngaged() {
  startCutterCam();
  while(cutSwitchIsOff()) {
    delay(postOperationWhileDelay);
  }
  stopWithDelay(CutterCamPin,postOperation1SecDelay); //postOperationShortDelay);
}

void runCutterUntilSwitchDisengaged() {
  startCutterCam();
  while(cutSwitchIsOn()) {
    delay(postOperationWhileDelay);
  }
}

void startCutterCam() {
  if (!cutterCamIsOn()) {
    startWithDelay(CutterCamPin,postOperationLongDelay);
  } else {
    Serial.println("  -  Cutter Cam was ON; ignored the start");
  }
}

boolean cutSwitchIsOff() {
  return (digitalRead(CutSwitchPin) == HIGH); 
}

boolean cutSwitchIsOn() {
  return (digitalRead(CutSwitchPin) == LOW); 
}

boolean cutterCamIsOn() {
  return (digitalRead(CutterCamPin) == HIGH);
}

/************************************
 *  U T I L I T Y    M E T H O D S  *
 ************************************/ 
void stopUsingPin(int pinValue) {
  if (digitalRead(pinValue) == HIGH) {
    digitalWrite(pinValue, LOW);
  }
}

void startUsingPin(int pinValue) {
  if (digitalRead(pinValue) == LOW) {
    digitalWrite(pinValue, HIGH);
  }
}

void stopWithDelay(int pin, int duration) {
  if (digitalRead(pin) == HIGH) {
    stopUsingPin(pin);
    delay(duration);
  }
}

void startWithDelay(int pin, int duration) {
  if (digitalRead(pin) == LOW) {
    startUsingPin(pin);
    delay(duration);
  }
}

void engageBallStop(int delayLength) {      
  startWithDelay(RodStopperPin, delayLength);
}

void retractBallStop(int delayLength) {
  stopWithDelay(RodStopperPin, delayLength);   
}

void advanceRod(int duration, int finalDelay) {
  Serial.println(" -  Advance Rod");
 // Serial.println(" -     Duration:" + duration);
  startWithDelay(RodPusherPin,duration);
  stopWithDelay(RodPusherPin,finalDelay);
}

void spinRodMotor() {
  startWithDelay(SpinMotorPin,10);
}

void stopRodMotor(int delayLength) {
  stopWithDelay(SpinMotorPin,delayLength);
}

void setTorchAndDelay(int angle, int delayLength) {
  torch.write(angle);   
  delay(delayLength);  
}

void operateTorch(int burnAngle, int duration, int finalDelay) {
  Serial.println(" -  Operate Torch");
//  Serial.println(" -     Burn Angle:" + burnAngle);
//  Serial.println(" -     Duration:" + duration);
  setTorchAndDelay(torchRestingAngle,100);         
  setTorchAndDelay(burnAngle,duration);
  setTorchAndDelay(torchRestingAngle,finalDelay);   
  Serial.println(" -  Operate Torch Complete.");
}