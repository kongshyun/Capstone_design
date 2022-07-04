

////////////데모 프로파일/////////////////

const float pi = 3.141592;              //
const float g = 9.81;                   // 중력가속도

const int servoPin_1 = 5;               // 모터 1
const int servoPin_2 = 6;               // 모터 2
const int servoPin_3 = 7;               // 모터 3
const int servoPin_4 = 8;               // 실험용 모터 


float a;                                // accel 현재 가속도
float theta;                            // 보정각도 프로파일 세타
double theta_x , theta_y, theta_z;      // 역기구학 통과한 모터1,2,3 각도

const float freq = 3003.003;            // 333Hz
const int minPulse = 500;               //
const int maxPulse = 2500;              //


unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임

float target_val_x;                     //가변저항 A0
float target_val_y;                     //가변저항 A1
float target_angle_x, target_angle_y;   //실험용 타겟x, 타겟y



double max_time_3ms=2.00;
double max_time_2ms=1.00;
double max_time_1ms=0.50;
double scale_3ms=0.02;
double scale_2ms=0.02;
double scale_1ms=0.01;
/*
double delay_3ms=19.3;
double delay_2ms=10.2;//짐벌얹었을때 딱 2m 나오는 delay값
double delay_1ms=9;*/
double sec=0.00;
double Back_time=500;
char CMD;


void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);//agv
  Wire.begin();
  MPU6050_setup();
  
  //pin 설정. 서보모터 쪽 출력(ouput) 설정
  pinMode(servoPin_1, OUTPUT);
  pinMode(servoPin_2, OUTPUT);
  pinMode(servoPin_3, OUTPUT);
  digitalWrite(servoPin_1, LOW);
  digitalWrite(servoPin_2, LOW);
  digitalWrite(servoPin_3, LOW);
  Serial.println("hello");
}


void loop()//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
{                                                             
//////////////////////////////                                                            
loop_start_time = millis();
//////////////////////////////
    
    //MPU6050_data_read(); 
    //Potentiometer(-30,30);
    //Plate_angle(-angle_x, -angle_y);
    //Plate_angle(target_val_x, target_val_y);
    
    Plate_angle(0, 0);
    
    //Motor_operation(servoPin_1, 90);
    //Motor_operation(servoPin_2, 90);
    //Motor_operation(servoPin_3, 90); 
  //motor_set();
  if(Serial.available()){
    CMD=Serial.read();
    if(CMD=='3'){
        Serial.println("---------------33333----------------");
        Agv_accel(3,max_time_3ms,scale_3ms,2000,100);
        sec=0;
        Serial3.println("co1=0");Serial3.println("co2=0");
        Serial.println("DONE");
    }
    if(CMD=='2'){
        Serial.println("---------------22222----------------");
        Agv_accel(2,max_time_2ms,scale_2ms,1000,50);
        Agv_decel(2,max_time_2ms,scale_2ms,1000,50);
    }
    if(CMD=='1'){
        Serial.println("---------------11111----------------");
        Agv_accel(1,max_time_1ms,scale_1ms,500,25);
        Agv_decel(1,max_time_1ms,scale_1ms,500,25);
    }
    if(CMD=='4'){
        Serial.println("--------------------BACK-----------");
        Agv_accel(4,max_time_1ms,scale_1ms,500,50);
        delay(Back_time);
        Agv_decel(4,max_time_1ms,scale_1ms,500,50);
    }         
  }     
  timecycle(); // 주기 계산 함수                          
                                                    
}//loop//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


//모터정지(초기화)
void motor_set(){
  Serial3.println("co1=1");//ch1,2모터 power on
  Serial3.println("co2=1");
  Serial3.println("mvc=0,0");//모터초기속도설정
}

void Plate_angle(float Roll, float Pitch){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
  
  Calculate_Roll_Pitch(Roll, Pitch);
  
  Motor_operation(servoPin_1, theta_x);
  Motor_operation(servoPin_2, theta_y);
  Motor_operation(servoPin_3, theta_z);
  
}//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
  double t;                            
  t = millis() - loop_start_time;
  dt = t;
  
  //Serial.print("loop time : ");
  //Serial.print(dt);
  //Serial.println();
                                       
}//timecycleㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

void Calculate_delay(int target_time, int number ){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                                                            //target_time (ms)
                                                            //number (횟수)  
  int cycle_end_time = millis();                            //사이클 끝나는 시각
  int cycle_time;                                           //측정된 1 사이클 시간
  int target_delay;                                         //원하는 딜레이 시간
  
  cycle_time = cycle_end_time - cycle_start_time;
  target_delay = (target_time/number)- cycle_time;   
 
  delay(target_delay);
                                                                        
}//Calculate_delay//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

void Potentiometer(int number_start , int number_end){//가변저항 함수ㅡㅡㅡㅡㅡ
  
  number_end = number_end*100;
  number_start = number_start*100;
  
  double val_1 = analogRead(A0);
  val_1 = map(val_1, 0, 1023 , number_start, number_end);
  double val_2 = analogRead(A1);
  val_2 = map(val_2, 0, 1023 , number_start, number_end);

  target_val_x = val_1/100;
  target_val_y = val_2/100;
   
}//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

void Motor_operation(int pin, int degree) {//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                                                         
  float hTime = 0;
  float lTime = 0;

  if (degree < 0) degree = 0;
  if (degree > 180) degree = 180;

  hTime = (minPulse + ((maxPulse - minPulse) / 180.0 * degree));
  lTime = freq - hTime;


  digitalWrite(pin, HIGH);
  delayMicroseconds(hTime);
  digitalWrite(pin, LOW);
  delayMicroseconds(lTime);
                                                             
}//Motor_operation//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


void Calculate_Roll_Pitch(float Target_roll, float Target_pitch) {//ㅡㅡㅡㅡㅡㅡ
  // Target_roll = 원하는 롤 값,  Target_pitch = 원하는 피치값                        
  // link_A = 링크 1의 길이 , link_B = 링크2의 길이                                     
                                                                                     
  float link_A1 = 40;
  float link_A2 = 40;
  float link_A3 = 40;
  float link_B1 = 61;
  float link_B2 = 61;
  float link_B3 = 61;
  float height_data = 67; 

  float Length_high, Length_low;

  Length_high = 110; // 윗면의 삼각형 한 변의 길이
  Length_low  = 35; // 아랫면의 삼각형 한 변의 길이
  
  Length_high = Length_high*sqrt(3)/2;
  Length_low  = Length_low*sqrt(3)/2;
  //이제부터 배열을 사용한다.
  float P[] = {0, 0, height_data}; //T(3 by 1  행렬임)

  float b1[] = { Length_high / sqrt(3),      0 ,              0}; //T
  float b2[] = { -Length_high / (2 * sqrt(3)),  Length_high / 2 ,  0}; //T
  float b3[] = { -Length_high / (2 * sqrt(3)), -Length_high / 2 ,  0}; //T
  float a1[] = { Length_low / sqrt(3),       0 ,              0}; //T
  float a2[] = { -Length_low / (2 * sqrt(3)),   Length_low / 2 ,   0}; //T
  float a3[] = { -Length_low / (2 * sqrt(3)),  -Length_low / 2 ,   0}; //T

  float x = Target_roll * (pi / 180); // 각도를 라디안으로 변환함
  float y = Target_pitch * (pi / 180);

  double R[3][3] = {  {cos(y), sin(x)*sin(y), cos(x)*sin(y)}, {0, cos(x), -sin(x)}, 
  { -sin(y), sin(x)*cos(y), cos(x)*cos(y)}  };

  int i, j, k; // for 문 변수 선언

  double q1[3] = {0, 0, 0} ; //T
  double q2[3] = {0, 0, 0} ; //T
  double q3[3] = {0, 0, 0} ; //T

  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      q1[i] += R[i][k] * b1[k]; // q1 = R*b1
      q2[i] += R[i][k] * b2[k]; // q2 = R*b2
      q3[i] += R[i][k] * b3[k]; // q3 = R*b3
    }
  }
  q1[2] += height_data;
  q2[2] += height_data;
  q3[2] += height_data;
  ////////////////////////////first//////////////////////////////////
  float x1 = q1[0] - a1[0];
  float x2 = q1[1] - a1[1];
  float x3 = q1[2] - a1[2];
  float x_data_1, y_data_1;
  y_data_1 = x3;
  x_data_1 = sqrt(pow(x1, 2) + pow(x2, 2));
  int dot_data_1 = x1 * a1[0] + x2 * a1[1] + x3 * a1[2]; // 내적
  if (dot_data_1 < 0) x_data_1 = - x_data_1;
  ////////////////////////////second///////////////////////////////////
  float y1 = q2[0] - a2[0];
  float y2 = q2[1] - a2[1];
  float y3 = q2[2] - a2[2];
  float x_data_2, y_data_2;
  y_data_2 = y3;
  x_data_2 = sqrt(pow(y1, 2) + pow(y2, 2));
  int dot_data_2 = y1 * a2[0] + y2 * a2[1] + y3 * a2[2]; // 내적
  if (dot_data_2 < 0) x_data_2 = - x_data_2;
  ////////////////////////////third///////////////////////////////////
  float z1 = q3[0] - a3[0];
  float z2 = q3[1] - a3[1];
  float z3 = q3[2] - a3[2];
  float x_data_3, y_data_3;
  y_data_3 = z3;
  x_data_3 = sqrt(pow(z1, 2) + pow(z2, 2));
  int dot_data_3 = z1 * a3[0] + z2 * a3[1] + z3 * a3[2]; // 내적
  if (dot_data_3 < 0) x_data_3 = - x_data_3;

  // double theta_x ,theta_y, theta_z; // 지역변수 아닌 전역변수로 선언하기 위해 위로 갑니다
    theta_x = length_to_motor_degree(x_data_1, y_data_1, link_A1, link_B1);
    theta_y = length_to_motor_degree(x_data_2, y_data_2, link_A2, link_B3);
    theta_z = length_to_motor_degree(x_data_3, y_data_3, link_A3, link_B3);

    int initial_angle_error_x = 0; 
    int initial_angle_error_y = -10;
    int initial_angle_error_z = -5;

    theta_x += initial_angle_error_x;//초기 모터 위치 값 에러 보정
    theta_y += initial_angle_error_y;
    theta_z += initial_angle_error_z;

    theta_x += 90; //모터가 세워져있기 때문에 
    theta_y += 90;
    theta_z += 90;   

    theta_x = constrain(theta_x, 30, 160); //위치 제한
    theta_y = constrain(theta_y, 30, 160);
    theta_z = constrain(theta_z, 30, 160);
                                                                
}/////////함수 끝//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


double length_to_motor_degree(double x_data, double y_data, double link_A,
double link_B) {
                                                                                                                  
  double thb, thbb, tha, thaa;                                                                                       
  double x = x_data;
  double y = y_data;
  double A = link_A;
  double B = link_B;
  double M = ((x * x) + (y * y) - (A * A) - (B * B)) / (2 * A * B); //중간 계산값

  tha = atan2(sqrt(abs(1 - pow(M, 2))), M);
  thb = atan2(y, x) - atan2(B * sin(tha), (A + (B * cos(tha))));
  thaa = atan2(-sqrt(abs(1 - pow(M, 2))), M);
  thbb = atan2(y, x) - atan2(B * sin(thaa), (A + (B * cos(thaa))));

  double theta1 = thb * 180 / pi;
  double theta2 = thbb * 180 / pi;

  // 어떤 각도를 선택할 건지 알고리즘 추가해야함.

  return theta1;                                                                                                   
                                                                                                               
}////////////////////////////////////////////////////////////////////


//AGV 가속함수
void Agv_accel(double vel_type,double Max_time,double scale,int target_time, int number){
  Serial3.println("co1=1");Serial3.println("co2=1");
  for(float sec=0;sec<Max_time;sec+=scale){
    cycle_start_time = millis();
    
    double Vel=VEL_TYPE(vel_type,sec);//속도공식
    a = a_TYPE(vel_type,sec);//가속도공식
    
    theta = atan(a/g)*180/3.14;
    Serial.println(theta);
    
    double RPM=Vel*60/(3.14*0.058);
    double cmd_rpm=RPM;//모터에 부여할 RPM
    Serial.print("RPM=  "); Serial.println(cmd_rpm);
    String cmd_str;//모터에 줄 속도 명령어
    cmd_str="mvc="+String(0)+","+String(cmd_rpm);
    
    Serial.println(cmd_str);
    Serial3.println(cmd_str);//----------------모터에명령통신-------
    
    Plate_angle(theta,0);
    
    Calculate_delay((int) target_time, (int) number );
  }
  Serial.print("\n");
}

//AGV 감속함수
void Agv_decel(double vel_type,double Max_time,double scale,int target_time, int number){
  
  for(float sec=Max_time;sec>0;sec-=scale){
      cycle_start_time = millis();
      
    double Vel=VEL_TYPE(vel_type,sec);//속도공식
    a = a_TYPE(vel_type,sec);//가속도공식
    theta = atan(a/g)*180/3.14;
    
    double RPM=Vel*60/(3.14*0.058);//모터에 부여할 RPM
    double cmd_rpm=RPM;
    Serial.print("RPM=  "); Serial.println(cmd_rpm,4);
    String cmd_str;//모터에 줄 속도 명령어
    cmd_str="mvc="+String(0)+","+String(cmd_rpm);
   
    Serial.println(cmd_str);
    Serial3.println(cmd_str);//----------------모터에명령통신-------
    
    Plate_angle(-theta,0);
      if(Serial.available()){
        char STOP=Serial.read();
        if(STOP=='0') return;
        Serial.print("\n");
      }
    Calculate_delay((int) target_time, (int) number );    
  }
  sec=0;
  Serial3.println("co1=0");Serial3.println("co2=0");
  Serial.println("DONE");
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


double a_TYPE(double vel_type,double sec){
  double a;
  if(vel_type==3){
    a= -((sec*sec*9)/4)+((sec*18)/4);
  }
  if(vel_type==2){
    a= -12*(sec*sec)+(sec*12);
  }
  if(vel_type==1){
    a= -(48*sec*sec)+(24*sec);
  }
  return a;
}
