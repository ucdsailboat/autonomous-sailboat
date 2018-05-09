//Davi's Wind Vane Code 
// Updated by JL 5/9/18

void wind_vel(){
  buttonState_ant = buttonState;
  buttonState = digitalRead(buttonPin);
  
  if (buttonState == HIGH && buttonState_ant == LOW) {
    endtime = millis();
    velocidade = (((float) 1)/(endtime-starttime)*1000.0)*6.6667;
    starttime = millis();
  }
}

float wind_dir(){
  int birutaValue;
  float soma;
  for(int j = 0; j < 10; j++){
    birutaValue = analogRead(birutaPin);
    soma += birutaValue;
  }
  birutaValue = soma/10;
  if(birutaValue >= 770 && birutaValue <= 810) {biruta = 0;}
  if(birutaValue >= 690 && birutaValue <= 730) {biruta = 22.5;}
  if(birutaValue >= 870 && birutaValue <= 920) {biruta = 45;}
  if(birutaValue >= 815 && birutaValue <= 850) {biruta = 67.5;}
  if(birutaValue >= 930 && birutaValue <= 960) {biruta = 90;}
  if(birutaValue >= 590 && birutaValue <= 615) {biruta = 112.5;}
  if(birutaValue >= 620 && birutaValue <= 640) {biruta = 135;}
  if(birutaValue >= 240 && birutaValue <= 260) {biruta = 157.5;}
  if(birutaValue >= 280 && birutaValue <= 300) {biruta = 180;}
  if(birutaValue >= 120 && birutaValue <= 135) {biruta = -157.5;}
  if(birutaValue >= 170 && birutaValue <= 200) {biruta = -135;}
  if(birutaValue >= 50 && birutaValue <= 74) {biruta = -112.5;}
  if(birutaValue >= 87 && birutaValue <= 100) {biruta = -90;}
  if(birutaValue >= 75 && birutaValue <= 86) {biruta = -67.5;}
  if(birutaValue >= 455 && birutaValue <= 475) {biruta = -45;}
  if(birutaValue >= 400 && birutaValue <= 420) {biruta = -22.5;}
  if(biruta < 0) {biruta = -biruta;}
  return biruta;
}