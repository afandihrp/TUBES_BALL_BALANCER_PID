#include <Servo.h>
#include <stdio.h>

Servo myservo;

const int trigPin = 8;
const int echoPin = 9;
const int servoPin = 6;

int min_constraint = -20;
int max_constraint = 20;

float PID_p, PID_i, PID_d, integral, derivative, PID_total, error, last_error, last_derivative;

float setpoint = 6;

float kp=1; 
float ki=0.05; 
float kd=3.6; 
int8_t offset = 4;

void setup() {

  Serial.begin(9600);
  myservo.attach(servoPin);
  myservo.write(90);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  


}



int8_t distances = 0;



void loop() 
{
  distances = getDistance();
  error = -(setpoint - distances);

  derivative = error - last_error;  

  if(distances<setpoint || distances>setpoint)
  {
    integral += error; 
  }
  else
  {
    integral = 0;
  }

  PID_total = (kp * error) + (kd * derivative) + (ki * integral);

  float servoangle = constrain(PID_total, min_constraint, max_constraint);

  Serial.print("setpoint:"+String(setpoint)+",");
  Serial.print("Jarak:");
  Serial.print(distances);
  Serial.print(",");

  // Serial.print("Error:"+String(error)+",");
  Serial.print("derivative:"+String(derivative)+",");
  Serial.print("pid_d:"+String(kd * derivative)+",");
  // Serial.print("integral:"+String(integral)+",");
  Serial.print("pid_i:"+String(ki * integral)+",");
  Serial.print("PID:"+String(PID_total));
  Serial.print(",");
  Serial.println("SERVO Angle:"+String(servoangle));
  myservo.write(servoangle+90);

  last_error = error;
  delay(205);
}

float getDistance() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  delay(1);
  return (duration * 0.034 / 2) - offset;
  
}
