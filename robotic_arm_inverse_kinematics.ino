#include <Adafruit_PWMServoDriver.h>

int xPotPin = A0;
int yPotPin = A1;
int zPotPin = A2;
// int writstPotPin = A3; //NOT IN USE
int handPotPin = A3;
int enableArmPin = 7;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2400
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

double x = 0;
double y = 0;
double z = 0;
int q1 = 0;
int q2 = 0;
int q3 = 0;

// int wristAngle = 0; //NOT IN USE
int handAngle = 0;

#define L1 104.25
#define L2 118.0

void setup() {
  Serial.begin(9600);

  pinMode(enableArmPin, INPUT);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

void loop() {
  if (digitalRead(enableArmPin)) {
    // Serial.println(digitalRead(enableArmPin));
    x = map(analogRead(xPotPin), 0, 1023, 78, 166);
    y = map(analogRead(yPotPin), 0, 1023, -96, 96);
    z = map(analogRead(zPotPin), 0, 1023, 0, 111);

    // wristAngle = map(analogRead(writstPotPin), 0, 1023, 0, 180); // NOT IN USE
    handAngle = map(analogRead(handPotPin), 0, 1023, 0, 180);
    // Serial.println(handAngle);
    // Serial.println();


    q1 = map(int(calculate_q1(x, y) * 180 / PI), -30, 30, 60, 120);
    // Serial.println(q1);

    q2 = int(calculate_q2(x, y, z) * 180 / PI);
    // Serial.println(q2);

    q3 = int(calculate_q3(x, y, z) * 180 / PI);
    // Serial.println(q3);

    Serial.print(q1);
    Serial.print(",");
    Serial.print(q2);
    Serial.print(",");
    Serial.println(180-q3);

    pwm.setPWM(0, 0, setAngle(q1));
    pwm.setPWM(5, 0, setAngle(180-q2));
    pwm.setPWM(1, 0, setAngle(q2));
    pwm.setPWM(2, 0, setAngle(180-q3));
    // pwm.setPWM(3, 0, setAngle(wristAngle)) // NOT IN USE;
    pwm.setPWM(4, 0, setAngle(handAngle));
  }
}

int setAngle(int angle){
  int pulse_width, analog_value;
  pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_width) / 1000000 * FREQUENCY * 4096);
  // Serial.println(analog_value);
  return analog_value;  
}

double calculate_q1(double x, double y){
  return atan(y / x);
}

double calculate_q2(double x, double y, double z){
  double c = x*x + y*y + z*z;
  return atan(z / sqrt(x*x + y*y)) + acos((L1*L1 + c - L2*L2) / (2*L1*sqrt(c)));
}

double calculate_q3(double x, double y, double z){
  return acos((L1*L1 + L2*L2 - (x*x + y*y + z*z)) / (2*L1*L2)) - (40.0 * PI / 180.0);
}