#include <Wire.h>

#define REF_OUT_A 2 // PD3
#define REF_OUT_B 3 // PD2
#define PENDULUM_ENCODER_PPR  2400


volatile long refEncoderValue = 0;
volatile long lastRefEncoded = 0;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float dt;
float theta, last_theta, theta_dot, last_theta_dot;
float control, u;
const float PI2 = 2.0 * PI;
byte* theta_bytes;
byte* theta_dot_bytes;
byte* payload;
void refEncoderHandler();

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  Serial.begin(9600);
  lastTimeMicros = 0L;
}



void loop()
{
  now = micros();
  dt = 1.0 * (now - lastTimeMicros) / 1000000;

  theta = getAngle(refEncoderValue, PENDULUM_ENCODER_PPR);
  theta_dot = (theta - last_theta) / dt;
  if (abs(theta_dot - last_theta_dot) > 100){
    theta_dot = last_theta_dot;
  }
  
  last_theta = theta;
  last_theta_dot = theta_dot;
  lastTimeMicros = now;
  
  theta_bytes = (byte*) &theta;
  theta_dot_bytes = (byte*) &theta_dot;

  Wire.beginTransmission(4); // transmit to device #4
  Wire.write(theta_bytes,4);
  Wire.write(theta_dot_bytes,4);
 
  Wire.endTransmission();    // stop transmitting
  delay(5);
}

float getAngle(long pulses, long ppr) {  
  float angle = (PI + PI2 * pulses / ppr);
  while (angle > PI) {
    angle -= PI2;
  }
  while (angle < -PI) {
    angle += PI2;
  }
  return angle;
}



void refEncoderHandler() {
  int MSB = (PIND & (1 << PD2)) >> PD2; //MSB = most significant bit
  int LSB = (PIND & (1 << PD3)) >> PD3; //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastRefEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    refEncoderValue++; //CW
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    refEncoderValue--; //CCW
  }

  lastRefEncoded = encoded; //store this value for next time  
}
