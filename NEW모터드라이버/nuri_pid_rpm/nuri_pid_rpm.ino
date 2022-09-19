#include <PIDController.h>
#include <Wire.h>
volatile long int encoder_pos = 0;
PIDController pos_pid; 
PIDController speed_pid; 
int motor_mode = 2; // 0 - none, 1 - position control, 2 speed control
long int last_encoder_pos = 0;
long int last_millis = 0;
int motor_value = 0;
int dir        = 0;
double SetPoint = 0;
float RPM = 0;

#define                Left   1
#define               Right   2
#define                Stop   0

#define         encoderPinA   2      // pin  2 
#define         encoderPinB   3      // pin  3 
#define               Brake   4      // pin  4
#define         MOTOR_1_PWM   5      // pin  5 
#define         MOTOR_2_PWM   6      // pin  6 //x
#define   MOTOR_1_Direction   7      // pin  7 
#define   MOTOR_2_Direction   8      // pin  8 //x

#define                  Kp_pos   20         // 위치 p
#define                  Ki_pos   0.1        // 위치 i
#define                  Kd_pos   2          // 위치 d 

#define                  Kp_rpm   5          // 속도 p
#define                  Ki_rpm   0.5     
#define                  Kd_rpm   1

const float ratio = 360./5./68.;// 360도,기어비,ppr 34*2 채널

/*
#define MOTOR1A 11
#define MOTOR1B 12
#define MOTOR2A 13
#define MOTOR2B 7
#define PWM1 10
#define PWM2 9
#define ENCODER1 2
#define ENCODER2 3
#define GREEN_LED 5
#define RED_LED 4
#define BUZZER 6
*/


void setup() {

  Serial.begin(115200);
  Wire.begin(20);
  Wire.onReceive(dataReceive);
  //attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder, RISING);
  // initialize digital pin 13 as an output.

   pinMode(encoderPinA, INPUT_PULLUP);
   attachInterrupt(0, doEncoderA, CHANGE);
   pinMode(encoderPinB, INPUT_PULLUP);
   attachInterrupt(1, doEncoderB, CHANGE);
   
   pinMode(    MOTOR_1_Direction, OUTPUT);  
   pinMode(    MOTOR_2_Direction, OUTPUT);  
   pinMode(          MOTOR_1_PWM, OUTPUT);  
   pinMode(          MOTOR_2_PWM, OUTPUT);
   pinMode(                Brake, OUTPUT);
   
     
   digitalWrite(  MOTOR_1_Direction,    HIGH);  
   digitalWrite(  MOTOR_2_Direction,    HIGH);
   digitalWrite(              Brake,     LOW);

   pos_pid.begin();    
   pos_pid.tune(Kp_pos, Ki_pos, Kd_pos);    
   pos_pid.limit(-255, 255);
   pos_pid.setpoint(0);

   speed_pid.begin();   
   speed_pid.tune(Kp_rpm, Ki_rpm, Kd_rpm);    
   speed_pid.limit(-255, 255);

}


void loop(){
  SetPoint = Potentiometer(-300,300);
    pos_pid.setpoint(SetPoint);
  speed_pid.setpoint(SetPoint);
  
  if(motor_mode == 1){
     float motorDeg = float(encoder_pos)*ratio;
     motor_value = pos_pid.compute(motorDeg);
     dir = motor_direction(motor_value);
     motor_control((dir > 0 )?Left:Right, min(abs(motor_value),255));

  }
  else if(motor_mode == 2){
    float rpm_speed = ((float(encoder_pos) - float(last_encoder_pos)) / 340) *60.0 * (1000/(millis()-last_millis));
    RPM = rpm_speed;
    motor_value = speed_pid.compute(rpm_speed);
    dir = motor_direction(motor_value);
    motor_control((dir > 0 )?Left:Right, min(abs(motor_value),255));
    last_encoder_pos = encoder_pos;
    last_millis = millis();
    delay(10);
    
  }

   Serial.print("Target degree : ");
   Serial.print(SetPoint);
   Serial.print(",");
   Serial.print("RPM : ");
   Serial.print(RPM);
   Serial.print(",");
   Serial.print("PID : ");
   Serial.print(motor_value);
   Serial.println();

  
}

/*void encoder(){

  if(digitalRead(encoderPinB) == HIGH){
    encoder_pos++;
  }else{
    encoder_pos--;
  }
}*/
void doEncoderA(){ 
  encoder_pos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;
}
void doEncoderB(){ 
  encoder_pos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;
}

void MotorStop(){
  digitalWrite(MOTOR_1_PWM, LOW);
  digitalWrite(MOTOR_2_PWM, LOW);
  pos_pid.setpoint(last_encoder_pos);
  motor_mode = 0;
}


void dataReceive(int data_length){
  int i = 0;
  
  char data[10]={};
  
  while(Wire.available()){
   data[i] = Wire.read();
   i++;
  }
  String dataString = String(data);
  if(dataString.equals("stop")){
    MotorStop();
  }else if(dataString.substring(0, 1).equals("s")){
      motor_mode = 2;
      int value = dataString.substring(1).toInt();
       speed_pid.setpoint(value); 
  }else if(dataString.substring(0, 1).equals("r")){
    motor_mode = 1;
     int value = dataString.substring(1).toInt();
     pos_pid.setpoint(last_encoder_pos + value);
  }else if(dataString.substring(0, 1).equals("l")){
    motor_mode = 1;
      int value = dataString.substring(1).toInt();
      pos_pid.setpoint(last_encoder_pos - value);
     //  digitalWrite(RED_LED, HIGH);
  }
}

void motor_control(int motor_direction, int velocity){

  int stop_pwm = 1;

  if((velocity < stop_pwm)|| motor_direction == Stop ){// stop
    //Serial.println("Motor Stop");
    //Serial.println();
    //pwmWrite(MOTOR_1_PWM, 0);
    //digitalWrite(    MOTOR_1_PWM,   HIGH);
    digitalWrite(            Brake,    HIGH);
      
  }

    if((velocity > stop_pwm) && motor_direction == Left){               // left
    //Serial.print("  Motor Left, ");
    //Serial.println(velocity);
    //Serial.println();
    digitalWrite(            Brake,    LOW);  
    digitalWrite(MOTOR_1_Direction,   HIGH);
    digitalWrite(MOTOR_2_Direction,   HIGH);
    //pwmWrite(MOTOR_1_PWM, velocity);
    digitalWrite(MOTOR_1_PWM,  HIGH);
    analogWrite (MOTOR_1_PWM, velocity);
  }

    if((velocity > stop_pwm) && motor_direction == Right){            // right
    //Serial.print("  Motor Right , ");
    //Serial.println(velocity);
    //Serial.println(); 
    digitalWrite(            Brake,    LOW);
    digitalWrite(MOTOR_1_Direction,    LOW);
    digitalWrite(MOTOR_2_Direction,    LOW);
    //pwmWrite(MOTOR_1_PWM, velocity);
    digitalWrite(MOTOR_1_PWM,  HIGH);
    analogWrite (MOTOR_1_PWM, velocity);
  }
  
  
}
int motor_direction(double pid){
  int Dir;
  if(pid > 0) Dir =  1;
  if(pid < 0) Dir = -1;
  return Dir;
}

float Potentiometer(int number_start , int number_end){
  
  number_start = number_start*100;
  number_end   = number_end*100;
  
  float val_1 = analogRead(A0);
  val_1 = map(val_1, 0, 1023 , number_start, number_end);

  return val_1/100;
}
