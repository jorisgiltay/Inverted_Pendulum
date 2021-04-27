#include <Wire.h>

#define REF_OUT_A 2 // PD3
#define REF_OUT_B 3 // PD2
#define MOTOR_ENCODER_PPR  2797
#define POSITION_LIMIT 0.18

#define PWM_PIN 10
#define DIR_PIN 8
#define SHAFT_R 0.011
#define BTN_PIN 11
#define LED_PIN 12

#define A 18.08
#define B 0.37
#define C 0.15


// 40cm (works okay)
#define Kx  3.05
#define Kv  3.4
#define Kth 22.5
#define Kw  4.4


//// 40cm trials works a bit faster on x
//#define Kx  9.6
//#define Kv  7.5
//#define Kth 32.5
//#define Kw  6.7

const float THETA_THRESHOLD = PI / 16;
bool moveleft = true;
bool from_control = false;
bool return_to_center = false;

volatile long refEncoderValue = 0; // for encoder motor
volatile long lastRefEncoded = 0;

volatile float theta;   //for i2c comm
volatile float theta_dot;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float x, v, dt, elapsed, RPM;
float last_x, last_v;
float control, u;
const float PI2 = 2.0 * PI;
int PWM;


void refEncoderHandler();

void setup() {
  delay(3000);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  // put your setup code here, to run once:

  TCCR2B = TCCR2B & B11111000 | B00000001;
  
  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  
  
  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  Serial.begin(57600);
  
  PWM = 0;
  analogWrite(PWM_PIN, PWM);
  lastTimeMicros = 0L;

}

void loop() {
  //Only loop this when the switch is turned on
  if(digitalRead(BTN_PIN) == 0){
    driveMotor(0);
    digitalWrite(LED_PIN, LOW);
  }
  else{
    digitalWrite(LED_PIN, HIGH);
    now = micros();
    dt = 1.0 * (now - lastTimeMicros) / 1000000;
    elapsed += dt;
  
    // Get States (note that the other states Theta and thetadot are received via I2C (receiveEvent))
    x = getCartDistance(refEncoderValue, MOTOR_ENCODER_PPR);
    v = (x - last_x) / dt;
    if (abs(x - last_x) > 100){
      v = last_v;
    }
  
    if (isControllable(theta) && fabs(x) < POSITION_LIMIT && return_to_center == false) {
      from_control = true;
      control = (Kx * x + Kv * v + Kth * theta + Kw * theta_dot);
      u = (control + A * v + copysignf(C, v)) / B;
      u = 255.0 * u / 12.0;
      driveMotor(saturate(u, 254));
    } else {
      driveMotor(0);
    }
  
    // Swing back up if LQR failed
    if (!isControllable(theta) && fabs(x) < POSITION_LIMIT && from_control == false && return_to_center == false){
  
      if(fabs(theta) > 2  && theta_dot >= 0){
          driveMotor(240);
      }
      else if (fabs(theta)> 2 && theta_dot <0){
          driveMotor(-240);
      }
    }
  
    // Return to center when reaching the endpoints
    if (return_to_center == true){
      while(x > 0.001){
        x = getCartDistance(refEncoderValue, MOTOR_ENCODER_PPR);
        driveMotor(-70);
        }
      while(x < -0.001){
        x = getCartDistance(refEncoderValue, MOTOR_ENCODER_PPR);
        driveMotor(70);
      }
        driveMotor(0);
        return_to_center = false;
  
     }
      
    // Boolean logic for the different scenarios
    if (fabs(x) >= POSITION_LIMIT){
      return_to_center = true;
    }
  
    if(fabs(theta_dot) <5 && fabs(theta) > 1){
      from_control = false;
    }
  
    last_x = x;
    last_v = v;
    lastTimeMicros = now;
  
     
    Serial.println(theta);
  //  Serial.print(",");
  //  Serial.println(theta_dot);
  //  Serial.print(",");
  //  Serial.print(x);
  //  Serial.print(",");
  //  Serial.println(v);
    delay(5);
  }
}

boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;
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

float saturate(float v, float maxValue) {
  if (fabs(v) > maxValue) {
    return (v > 0) ? maxValue : -maxValue;
  } else {
    return v;
  }
}

void driveMotor(float u) {
  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));
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
  
    memcpy(&theta, &theta_bytes, sizeof(theta));
    memcpy(&theta_dot, &theta_dot_bytes, sizeof(theta_dot));
}
