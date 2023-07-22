#include <Wire.h>
#define CALIBRATE_SAMPLE 2000
#define THROTTLE_CUTOFF 1000
//#define MOTOR_DEBUG 1
//#define DEBUG 1                                                                       // This is used for debug purpose, selective compilation

// PCINT1 for throttle
// PCINT2 for Roll
// PCINT3 for Pitch
// PCINT4 for Yaw 

// D7 = PD7 = motor1
// D6 = PD6 = motor2
// D5 = PD5 = motor3
// D4 = PD4 = motor4

float RateRoll, RatePitch, RateYaw = 0;                                                  // these are used to store the measured angular velocity
float calibrateRoll, calibratePitch, calibrateYaw = 0;                                   // these are used for calibration


uint8_t throttle_prev, roll_prev, pitch_prev, yaw_prev = 0;                              // these are used to detect the rising and falling edge
uint32_t throttle_time, roll_time, pitch_time, yaw_time = 0;
uint32_t throttle_con , roll_con, pitch_con, yaw_con = 0;
uint32_t current_micro = 0;                                                              // these timer value is used to calculate the pwm HIGH width time

/* Variables used for PID calculation*/
uint32_t current_loop_micro, prev_loop_micro, loop_time = 0;                                                         // these are used for PID calculation

float pRoll = 2, pPitch = pRoll, pYaw = 0;                                                                // defining PID gains for Roll, Pitch and Yaw
float iRoll = 0, iPitch = iRoll, iYaw = 0;
float dRoll = 0, dPitch = dRoll, dYaw = 0;

float rollError, prevRollError, pitchError, prevPitchError, yawError, prevYawError = 0;
float i_roll, i_pitch, i_yaw = 0;                                                                        // used for integration that is for summation of previous errors 


float setRoll, setPitch, setYaw = 0;                                                                     // set point obtained from the reciever

int16_t roll_input, pitch_input, yaw_input = 0;
int16_t throttle_input = 0;                                                                             // throttle_con keeps getting updated by the ISR hence using this Var to keep a single value with in a loop                                 

// variable used to control the output
int16_t motor1_output, motor2_output, motor3_output, motor4_output = 1000;
uint32_t pulseStartMicro, checkMicro = 0; 

// function prototypes 
ISR(PCINT0_vect);

void gyro_signals();
void calibrate_gyro();

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("***Drone Controller debug mode***");
  #elif MOTOR_DEBUG
    Serial.begin(9600);
    Serial.println("***Drone Motor output debug mode***");
  #endif
  
  Wire.setClock(400000);
  Wire.begin();
  delay(150);
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);                        // selecting the internal 8MHz oscillator of MPU
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                       // register 26 CONFIG
  Wire.write(0x05);                       // setting the filter cut of to 5 HZ
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                       // register 27  GYRO_CONFIG
  Wire.write(0x8);                        // setting the full scale to +-500 degree per second
  Wire.endTransmission(); 

  // this code section is used to get the reference Gyro value
  for (uint16_t i = 0; i < CALIBRATE_SAMPLE ; i++)
  {
    calibrate_gyro();
    delay(1);  
  }
  
  calibrateRoll = calibrateRoll/CALIBRATE_SAMPLE;
  calibratePitch = calibratePitch/CALIBRATE_SAMPLE;
  calibrateYaw = calibrateYaw/CALIBRATE_SAMPLE;
  
  #ifdef DEBUG
    Serial.println(calibrateRoll);
    Serial.println(calibratePitch);
    Serial.println(calibrateYaw);
  #endif
  
  // This section of code is 
  cli();
  DDRB = 0;                 // 
  PORTB = 0;                // setting the pull down resistor 
  PCICR = 0x01;            // setting the pin change interrupt for 1 to 5
  PCMSK0 = 0b00011110;
  sei();
  delay(20);

  // this section is used to set up the ports for motor output
  DDRD |= 0b11111100;            // setting D7,D6,D5,D4,D4 and D3 as output
  PORTD &= !(0b11111100);           // setting the motor output to 0 and for the leds
  
  gyro_signals();
  // controller only starts if the throttle is set at lowest point at start
  PORTD |= 0b00001000;
  while((throttle_con < 1000)||(throttle_con>1050))
  {
    gyro_signals();
    delay(10);
  }
  PORTD &= !(0b00001000);
  PORTD |= (1<<PORTD2);
   
  prev_loop_micro = micros();
}

void loop() {
  gyro_signals();
  setRoll = 0.15*((float)roll_con-1500.0);
  setPitch = 0.15*((float)pitch_con-1500.0);
  setYaw = 0.15*((float)yaw_con-1500.0);
  throttle_input = throttle_con;
  
  rollError = setRoll - RateRoll;
  pitchError = setPitch - RatePitch;
  yawError = setYaw - RateYaw;
  
  current_loop_micro = micros(); 
  loop_time = (current_loop_micro-prev_loop_micro)/1000;
  prev_loop_micro = current_loop_micro;
  
  i_roll += iRoll*rollError*loop_time;
  i_pitch += iPitch*pitchError*loop_time;
  i_yaw += iYaw*yawError*loop_time;

  if(i_roll > 400) i_roll = 400; else if(i_roll < -400) i_roll = -400;
  if(i_pitch > 400) i_pitch = 400; else if(i_pitch < -400) i_pitch = -400;
  if(i_yaw > 400) i_yaw = 400; else if(i_yaw < -400) i_yaw = -400; 
  
  roll_input = (pRoll*rollError) + i_roll + dRoll*(rollError-prevRollError)/loop_time; 
  pitch_input = (pPitch*pitchError) + i_pitch + dPitch*(pitchError-prevPitchError)/loop_time;
  yaw_input = (pYaw*yawError) + i_yaw + dYaw*(yawError-prevYawError)/loop_time;

  if (roll_input > 400) roll_input = 400; else if(roll_input < -400) roll_input = -400;
  if (pitch_input > 400) pitch_input = 400; else if(pitch_input < -400) pitch_input = -400;
  if (yaw_input > 400) yaw_input = 400; else if(yaw_input < -400) yaw_input = -400;
  
  prevRollError = rollError;
  prevPitchError = pitchError;
  prevYawError = yawError;

  if ( throttle_input >  1800) throttle_input = 1800;
  motor1_output = throttle_input - roll_input - pitch_input - yaw_input; 
  motor2_output = throttle_input - roll_input + pitch_input + yaw_input;
  motor3_output =throttle_input + roll_input + pitch_input - yaw_input;
  motor4_output = throttle_input + roll_input - pitch_input + yaw_input;

  if (motor1_output > 2000) motor1_output = 2000; else if(motor1_output<1180) motor1_output = 1180;
  if (motor2_output > 2000) motor2_output = 2000; else if(motor2_output<1180) motor2_output = 1180;
  if (motor3_output > 2000) motor3_output = 2000; else if(motor3_output<1180) motor3_output = 1180;
  if (motor4_output > 2000) motor4_output = 2000; else if(motor4_output<1180) motor4_output = 1180;

  #ifdef MOTOR_DEBUG
    Serial.print("Yaw =");
    Serial.println(setYaw);
    /*
    Serial.print("Motor 1=");
    Serial.print(motor1_output);
    Serial.print(", Motor 2=");
    Serial.print(motor2_output);
    Serial.print(", Motor 3=");
    Serial.print(motor3_output);
    Serial.print(", Motor 4=");
    Serial.println(motor4_output); */
  #endif
  
  if (throttle_input < 1015)
  {
    rollError, prevRollError, pitchError, prevPitchError, yawError, prevYawError = 0;
    i_roll, i_pitch, i_yaw = 0;
    
    PORTD |= 0xf0;                                                                                   // set all the output high
    pulseStartMicro = micros();
    checkMicro = pulseStartMicro;
    while((checkMicro-pulseStartMicro) < THROTTLE_CUTOFF) checkMicro = micros();
    PORTD &= 0x0F;
  }
  else{
    
    PORTD |= 0xf0;
    pulseStartMicro = micros();
    while(PIND&0xF0)
    {
      checkMicro = micros();
      if(checkMicro-pulseStartMicro >= motor1_output) PORTD &= 0b01110100;
      if(checkMicro-pulseStartMicro >= motor2_output) PORTD &= 0b10110100; 
      if(checkMicro-pulseStartMicro >= motor3_output) PORTD &= 0b11010100; 
      if(checkMicro-pulseStartMicro >= motor4_output) PORTD &= 0b11100100;  
    }
  }
  
  #ifdef DEBUG
    delay(200);
    Serial.print("Throttle =");
    Serial.print(throttle_con);
    Serial.print(", Roll =");
    Serial.print(roll_con);
    Serial.print(", Pitch =");
    Serial.print(pitch_con);
    Serial.print(", Yaw =");
    Serial.println(yaw_con);
    Serial.println("**-----------**");
    Serial.print("Roll =");
    Serial.print(RateRoll);
    Serial.print(", Pitch =");
    Serial.print(RatePitch);
    Serial.print(", Yaw =");
    Serial.println(RateYaw);
  #endif
}



void gyro_signals()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x43);                         // starting address of register where the gyro measurement is stored                
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);                 // requesting 6 bytes of data from the MPU6050
  
  int32_t GyroX=Wire.read()<<8 | Wire.read();
  int32_t GyroY=Wire.read()<<8 | Wire.read();
  int32_t GyroZ=Wire.read()<<8 | Wire.read();
  
  RateRoll=(float)GyroX/65.5-calibrateRoll;
  RatePitch=(float)GyroY/65.5- calibratePitch;
  RateYaw=(float)GyroZ/65.5-calibrateYaw;
}

void calibrate_gyro()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x43);                         // starting address of register where the gyro measurement is stored                
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);                 // requesting 6 bytes of data from the MPU6050
  
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  
  calibrateRoll += (float)GyroX/65.5;
  calibratePitch += (float)GyroY/65.5;  
  calibrateYaw += (float)GyroZ/65.5;
}

ISR(PCINT0_vect){
  current_micro = micros();
    // for throtle control
    if( (PINB&0x02) && (!throttle_prev) ){
        throttle_time = current_micro;
        throttle_prev = 1;
    }
    else if( !(PINB&0x02) && throttle_prev ){
        throttle_con = current_micro-throttle_time;
        throttle_prev = 0;
    }
    
    // for roll control
    if( (PINB&0x04) && (!roll_prev) ){
        roll_time = current_micro;
        roll_prev = 1;
    
    } 
    else if( !(PINB&0x04) && roll_prev ){
        roll_con = current_micro-roll_time;
        roll_prev = 0;
    } 
    
    // for pitch control
    if( (PINB&0x08) && (!pitch_prev) ){
        pitch_time = current_micro;
        pitch_prev = 1;
    
    }
    else if( !(PINB&0x08) && pitch_prev ){
        pitch_con = current_micro-pitch_time;
        pitch_prev = 0;
    } 
    
    // for yaw control
    if( (PINB&0x10) && (!yaw_prev) ){
        yaw_time = current_micro;
        yaw_prev = 1;
    
    }
    else if( !(PINB&0x10) && yaw_prev ){
        yaw_con = current_micro-yaw_time;
        yaw_prev = 0;
    }  
}
