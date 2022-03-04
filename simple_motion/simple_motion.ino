void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);  Serial1.println("co=1");
  Serial1.println("co2=1");
  //Serial3.begin(115200);
} 

void loop() {
  delay(5000);
  /*
  Serial3.println("co=1");
  Serial3.println("co2=1");
  Serial3.println("mvc=200,200");
  */

  Serial1.println("mvc=50,100");
  delay(5000);
  Serial1.println("co=7");
  Serial1.println("co2=7");
  Serial1.println("v1");
  Serial1.println("v2");

  if(Serial.available()){
    Serial1.println("mvc=200,0");
    delay(2000);
    Serial1.println("mvc=100,0");
    delay(2000);
    Serial1.println("mvc=0,100");
    delay(500);
  }
  
}
