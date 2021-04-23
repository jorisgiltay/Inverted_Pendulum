#include <Wire.h>

#define REF_OUT_A 2 // PD3
#define REF_OUT_B 3 // PD2
#define PENDULUM_ENCODER_PPR  2000


volatile long refEncoderValue = 0;
volatile long lastRefEncoded = 0;
volatile float theta2;
volatile float theta_dot2;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float v, dt;
float theta, last_theta, theta_dot, last_theta_dot;
float control, u;
const float PI2 = 2.0 * PI;


void refEncoderHandler();

void setup() {
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  // put your setup code here, to run once:

  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  Serial.begin(9600);
  lastTimeMicros = 0L;
}

void loop() {
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

  Serial.print(theta);
  Serial.print(",");
  Serial.print(theta2);
  Serial.print(",");
  Serial.print(theta_dot);
  Serial.print(",");
  Serial.println(theta_dot2);
  

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

void receiveEvent(int howMany)
{
  //Serial.println(howMany);
  uint8_t total_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // fill this array with the four bytes you received
  uint8_t theta_bytes[4] = {0,0,0,0};
  uint8_t theta_dot_bytes[4] = {0,0,0,0};
  for (int i = 0; i<8; i++){
    total_bytes[i] = Wire.read();
  }
  for (int k = 0; k <4; k++){
  theta_bytes[k] = total_bytes[k];
  theta_dot_bytes[k] = total_bytes[k+4];
  }
  
    memcpy(&theta2, &theta_bytes, sizeof(theta2));
    memcpy(&theta_dot2, &theta_dot_bytes, sizeof(theta_dot2));
}
