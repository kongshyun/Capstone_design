
void setup() {
  //Serial3.begin(115200);
  Serial.begin(115200);
  //Serial3.println("co1=1");
  //Serial3.println("co2=1");
}

volatile double v1,v2;       
//void extract_velocity_Left();
double radius=0.0325;
void loop() {
  // put your main code here, to run repeatedly:
  String recv_str3 = "";
//Serial3.println("v1");
//    do {
//      recv_str3 += (char)Serial3.read();
//    } while (Serial3.available());
//    Serial.print(recv_str3);
//Serial3.println("v2");
//    do {
//      recv_str3 += (char)Serial3.read();
//    } while (Serial3.available());
//    Serial.print(recv_str3);

//Serial3.println("mvc=100,100");
//extract_velocity_Left();
//extract_velocity_Right();

  extract_velocity();
  Serial.print("\n");
  
  delay(1000);

}
/*
void extract_velocity_Right() 
{
  String recv_str3 = "";
  String str_v1, str_v2;
  bool positive;
  //Serial3.println("v");
  delay(5); ////데이터 송신후 수신 delay
  do {
    recv_str3 += (char)Serial.read();
  } while (Serial.available());
  //'=', '.' 인덱스 넘버 찾기
  int find_equal = recv_str3.indexOf('=', 0); //from 0
  int find_dot = recv_str3.indexOf('.', 0);
  int find_positive = recv_str3.indexOf('-', 0);
  //속도 값 추출
  for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
  if (find_positive < 0) positive = true;
  else if (find_positive > 0) positive = false;
  double v1_1 = str_v1.toInt();
  double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
  v1 = positive ? v1_1 + v1_2 : v1_1 - v1_2;//rpm
  double velocity_Right = v1*radius*2*3.141592/60;//속도ms
  Serial.print("v1= "); Serial.println(v1);
}

void extract_velocity_Left()
{
    String recv_str3 = "";
    String str_v1, str_v2;
    bool positive;
    //Serial3.println("v2");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial.read();
    } while (Serial.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) 
    {str_v1 += recv_str3[i];}
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v2 = positive ? v1_1 + v1_2 : v1_1 - v1_2;//rpm
    double velocity_Left = v2*radius*2*3.141592/60;//속도ms
    Serial.print("v2= ");Serial.println(v2);
}
*/

void extract_velocity() 
{
  String recv_str3 = "";
  String str_v1, str_v2;
  bool positive;
  //Serial3.println("v");
  delay(50); ////데이터 송신후 수신 delay
  
  do {
    recv_str3 += (char)Serial.read();
  } while (Serial.available());
  
  //'=', '.' 인덱스 넘버 찾기
  int find_equal = recv_str3.indexOf('=', 0); //from 0
  int find_dot = recv_str3.indexOf('.', 0);
  int find_positive = recv_str3.indexOf('-', 0);
  int find_comma=recv_str3.indexOf(',',0);
  
  //속도 값 추출
  for( int i = find_equal + 1; i < find_comma; i++) {str_v1 += recv_str3[i];}
  for( int i= find_comma+1;recv_str3.length();i++){str_v2 += recv_str3[i];}
  
  if (find_positive < 0) positive = true;
  else if (find_positive > 0) positive = false;
  
  double v1_1 = str_v1.toInt();
  double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;

  double v2_1 = str_v1.toInt();
  double v2_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
  
  v1 = positive ? v1_1 + v1_2 : v1_1 - v1_2;//rpm
  v2 = positive ? v2_1 + v2_2 : v2_1 - v2_2;//rpm
  
  double velocity_Right = v1*radius*2*3.141592/60;//속도ms
  
  Serial.print("v1= "); Serial.println(v1);
  Serial.print("v2= "); Serial.println(v2);
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
