
// Define the pins of the trigger pins and echo pins 
const int trigPin = 2;
const int echoPin = 6;

#define en1 3 // enable for left motor
#define en2 5 // enable for right motor
#define in1 7 
#define in2 8
#define in3 12
#define in4 13

float duration, cm;

void setup() {
  // The trigger pin will output an ultrasonic
  // signal from the speaker
  pinMode(trigPin, OUTPUT);

  // The echo pin will receive the reflected signal
  // and input it back to the IDE
  pinMode(echoPin, INPUT);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);

  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  pinMode(in3, INPUT);
  pinMode(in4, INPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);

  Serial.begin(9600);
  
   // put your setup code here, to run once:
  Serial.begin(9600);
}

void setForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW); 
}

void setBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH); 
}

void moveCar(int speed) {
  if (speed < 0) {
    speed *= -1;
    setBackward();
  }
  else {
    
    setForward();
  }
  if (speed > 30) {
    speed = 30;
  }
  analogWrite(en1, 220+speed);
  analogWrite(en2, 220+speed);  
}

int Kp = 3;
int Ki = -0.5;
int Kd = -0.5;

double lastError = 0;
double cumError;

void loop() {
  // put your main code here, to run repeatedly:
  // Define the duration beetween transmitted and
  // recieved
  
  //------- UltraSound -------
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //Send pulse
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Recieve pulse
  cm = (duration/2)*0.0343;
  delay(1);
  Serial.println(cm); 
  delay(10); // Delay in between samples

  double error = 15 - cm;
  cumError += error * 10;
  double rateError = (error-lastError)/10;
  lastError = error;
  
  int output = Kp * error + Ki * cumError + Kd * rateError;

  moveCar(output);
  
}
