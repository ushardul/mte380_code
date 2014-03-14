void setup (){
  Serial.begin (9600);
}

int main (){
  Serial.println (analogRead (A0));
}
