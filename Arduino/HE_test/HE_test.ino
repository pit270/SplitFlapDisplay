int v_out = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  v_out = analogRead(A0);
  if(v_out == 0.0)
    Serial.println(millis());
  delay(10);
}
