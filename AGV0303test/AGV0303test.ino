void setup(){
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial3.println("mvc=0,0");
}

void loop(){
  delay(500);
  Serial3.println("co1=1");
  Serial3.println("co2=1");
  Serial.println("NO Signal");
  char vel;
  if(Serial.available()){
    vel=Serial.read();
    if(vel=='e') {
      Serial.println("rpm=350");
      Serial3.println("mvc=-350,350");
    }
    else if (vel=='d'){
      Serial.println("rpm=200");
      Serial3.println("mvc=-200,200");
    }
    
    else if (vel=='c'){
      Serial.println("rpm=100");
      Serial3.println("mvc=-100,100");
    }
    else if (vel=='b'){
      Serial.println("rpm=50");
      Serial3.println("mvc=-50,50");
    }
    else if(vel=='a'){
      Serial.println("STOP");
      Serial3.println("mvc=0,0");
    }
    else if(vel=='1'){
      Serial3.println("mvc=20,-10");
    }
  }
}
