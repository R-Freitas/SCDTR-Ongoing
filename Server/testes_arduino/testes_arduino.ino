void setup() {
  Serial.begin(9600);
  Serial.println("Begin Setup");
  pinMode(LED_BUILTIN, OUTPUT);
  

}

void loop() {
    if(Serial.available()> 0){
      String data = "";
      while(Serial.available())
      {
        data += (char) Serial.read();
      }
  
    
  
      if (data[0] == 'A')
      {
        digitalWrite(LED_BUILTIN,HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN,LOW);
      }
      Serial.println((String) data);
    }


}
