/*
  AGV_0417_accel_decel_코드
  가속함수, 등속함수, 감속함수가 있다.
  매개변수
  -가속&감속함수(속도, 최대시간, 명령간격);
  -등속함수(속도, 유지시간);
  가속함수가 되는 동안 STOP명령을 주면 그때의 속도에서 그대로 감속된다.
*/

double max_time_3ms=2;
double max_time_2ms=1;
double max_time_1ms=0.5;
double scale_3ms=0.02;
double scale_2ms=0.01;
double scale_1ms=0.01;
double delay_3ms=18;
double delay_2ms=8;
double delay_1ms=9;
double sec=0;
char CMD;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop(){
  motor_set();
  if(Serial.available()){
    CMD=Serial.read();
    motor_set();
    switch(CMD){
      case '3':
        Serial.println("---------------33333----------------");
        Agv_accel(3,max_time_3ms,scale_3ms,delay_3ms);
        Agv_constant(3,0);
        Agv_decel(3,max_time_3ms,scale_3ms,delay_3ms);
        break;
      case '2':
        Serial.println("---------------22222----------------");
        Agv_accel(2,max_time_2ms,scale_2ms,delay_2ms);
        Agv_constant(2,10);
        Agv_decel(2,max_time_2ms,scale_2ms,delay_2ms);
        break;
      case '1':
        Serial.println("---------------11111----------------");
        Agv_accel(1,max_time_1ms,scale_1ms,delay_1ms);
        Agv_constant(1,0);
        Agv_decel(1,max_time_1ms,scale_1ms,delay_1ms);
        break;
      case '4':
        Serial.println("--------------------BBBB-----------");
        Agv_accel(4,max_time_1ms,scale_1ms,delay_1ms);
        Agv_constant(1,0);
        Agv_decel(4,max_time_1ms,scale_1ms,delay_1ms);
        break;        
    }
  }
}


//AGV 가속함수
void Agv_accel(double vel_type,double Max_time,double scale,double delay_time){
  Serial3.println("co1=1");Serial3.println("co2=1");
  while (sec <= Max_time){
    double Vel=VEL_TYPE(vel_type,sec);//속도공식
    double RPM=Vel*60/(3.14*0.065);//모터에 부여할 RPM
    double cmd_rpm=RPM/3.0;//AGV벨트풀리 3:1이므로 실제 모터에 부여할 RPM은 /3해야함.
    Serial.print("RPM=  "); Serial.println(cmd_rpm,4);//소수점 4자리까지표시
    String cmd_str;//모터에 줄 속도 명령어
    cmd_str="mvc="+String(-cmd_rpm)+","+String(cmd_rpm);
    //Serial.println(cmd_str);
    Serial3.println(cmd_str);//----------------모터에명령통신-------
  
    sec=sec+scale;delay(delay_time);
    if(Serial.available()){
      char STOP=Serial.read();
      if(STOP=='0') return;
      Serial.print("\n");
    }
  }
  
}

//AGV 감속함수
void Agv_decel(double vel_type,double Max_time,double scale,double delay_time){
  while (sec >0){
    double Vel=VEL_TYPE(vel_type,sec);//속도공식
    double RPM=Vel*60/(3.14*0.065);//모터에 부여할 RPM
    double cmd_rpm=RPM/3.0;//AGV벨트풀리 3:1이므로 실제 모터에 부여할 RPM은 /3해야함.
    Serial.print("RPM=  "); Serial.println(cmd_rpm,4);//소수점 4자리까지표시
    String cmd_str;//모터에 줄 속도 명령어
    cmd_str="mvc="+String(-cmd_rpm)+","+String(cmd_rpm);
    //Serial.println(cmd_str);
    Serial3.println(cmd_str);//----------------모터에명령통신-------
    sec=sec-scale; delay(delay_time);
    //'3'누르면 멈춤
    if(Serial.available()){
      char STOP=Serial.read();
      if(STOP=='0') return;
      Serial.print("\n");
    }
  }
  sec=0;
  Serial3.println("co1=0");Serial3.println("co2=0");
  Serial.println("DONE");
}

//AGV 등속함수

void Agv_constant(double vel_type,double constant_time)
{
  double Vel=VEL_TYPE(vel_type,sec);//속도공식
  double RPM=Vel*60/(3.14*0.065);//모터에 부여할 RPM
  double cmd_rpm=RPM/3.0;//AGV벨트풀리 3:1이므로 실제 모터에 부여할 RPM은 /3해야함.
  String cmd_str;//모터에 줄 속도 명령어
  cmd_str="mvc="+String(-cmd_rpm)+","+String(cmd_rpm);
  //Serial.println(cmd_str);
  Serial3.println(cmd_str);//----------------모터에명령-------
  Serial.println("  Constant!!  ");
  delay(constant_time);
  Serial.println("  DECEL!!  ");
}

//모터정지(초기화)
void motor_set()
{
  Serial3.println("co1=1");//ch1,2모터 power on
  Serial3.println("co2=1");
  Serial3.println("mvc=0,0");//모터초기속도설정
}

//속도프로파일공식_매개변수(속도, 시간)
double VEL_TYPE(double vel_type,double sec){
  double velocity;
  if(vel_type==3){
    velocity=-((sec*sec*sec*3)/4)+((sec*sec*9)/4);
  }
  if(vel_type==2){
    velocity=-((sec*sec*sec)*4)+((sec*sec*6));
  }
  if(vel_type==1){
    velocity=-(16*sec*sec*sec)+(12*sec*sec);
  }
  if(vel_type==4){
    velocity=-(-(16*sec*sec*sec)+(12*sec*sec));
  }
  return velocity;
}
