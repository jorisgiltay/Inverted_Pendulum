#include <Wire.h>

#define REF_OUT_A 2 // PD3
#define REF_OUT_B 3 // PD2
#define MOTOR_ENCODER_PPR  2797

#define PWM_PIN 10
#define DIR_PIN 8
#define SHAFT_R 0.011

volatile long refEncoderValue = 0; // for encoder motor
volatile long lastRefEncoded = 0;

volatile float phi;   //for i2c comm
volatile float phi_dot;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float x, v, dt, elapsed, RPM;
float last_x, last_v;
float control, u;
const float PI2 = 2.0 * PI;
int PWM;
int i = 0; 

void refEncoderHandler();

void setup() {
 // Wire.begin(4);                // join i2c bus with address #4
 // Wire.onReceive(receiveEvent); // register event
  // put your setup code here, to run once:

  TCCR2B = TCCR2B & B11111000 | B00000010;

  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  Serial.begin(57600);
  
  PWM = 125;
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, PWM);
  lastTimeMicros = 0L;

}

void loop() {
  now = micros();
  dt = 1.0 * (now - lastTimeMicros) / 1000000;
  i++;


  x = getCartDistance(refEncoderValue, MOTOR_ENCODER_PPR);
  v = (x - last_x) / dt;
  if (abs(x - last_x) > 100){
    v = last_v;
  }
  
  last_x = x;
  last_v = v;
    lastTimeMicros = now;

  elapsed += dt;
  

//  if (elapsed > 1.0 && elapsed <2.0){
//    analogWrite(PWM_PIN, 127);
//    digitalWrite(DIR_PIN, HIGH);
//  }
//  else if (elapsed > 2.0 && elapsed <3.0){
//    analogWrite(PWM_PIN, 127);
//    digitalWrite(DIR_PIN, LOW);
//  }
//  else if (elapsed >3.0 && elapsed <4.0){
//    analogWrite(PWM_PIN, 127);
//    digitalWrite(DIR_PIN, HIGH);
//  }
//  else if (elapsed > 4.0 && elapsed <5.0){
//    analogWrite(PWM_PIN, 127);
//    digitalWrite(DIR_PIN, HIGH);
//  }
 

  if(elapsed <10){
  Serial.print(i);
  Serial.print(",");
  Serial.print(-PWM);
  Serial.print(",");
  Serial.print(micros());
  Serial.print(",");
  Serial.println(x,6);
  }
   else{
    analogWrite(PWM_PIN, 0);
  }
 
  
//
//  Serial.println(theta);
//  Serial.print(",");
//  Serial.print(phi);
//  Serial.print(",");
//  Serial.println(theta_dot);
//  Serial.print(",");
//  Serial.println(phi_dot);
  delay(5);
 
  
}

float getCartDistance(long pulses, long ppr) {
  return 2.0 * PI * pulses / MOTOR_ENCODER_PPR * SHAFT_R;
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

//void receiveEvent(int howMany)
//{
//  //Serial.println(howMany);
//  uint8_t total_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // fill this array with the four bytes you received
//  uint8_t phi_bytes[4] = {0,0,0,0};
//  uint8_t phi_dot_bytes[4] = {0,0,0,0};
//  for (int i = 0; i<8; i++){
//    total_bytes[i] = Wire.read();
//  }
//  for (int k = 0; k <4; k++){
//  phi_bytes[k] = total_bytes[k];
//  phi_dot_bytes[k] = total_bytes[k+4];
//  }
//  
//    memcpy(&phi, &phi_bytes, sizeof(phi));
//    memcpy(&phi_dot, &phi_dot_bytes, sizeof(phi_dot));
//}
