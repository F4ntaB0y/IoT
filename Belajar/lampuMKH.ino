int ledm = 8;
int ledk = 9;
int ledh = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ledm, OUTPUT);
  pinMode(ledk, OUTPUT);
  pinMode(ledh, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(ledm, HIGH);
digitalWrite(ledk, LOW);
digitalWrite(ledh, LOW);
Serial.println(" ");
Serial.println("Merah Menyala");
Serial.println("Kungin Mati");
Serial.println("Hijau Mati");
delay(3000);

digitalWrite(ledm, LOW);
digitalWrite(ledk, HIGH);
digitalWrite(ledh, LOW);
Serial.println(" ");
Serial.println("Merah Mati");
Serial.println("Kungin Menyala");
Serial.println("Hijau Mati");
delay(1000);

digitalWrite(ledm, LOW);
digitalWrite(ledk, LOW);
digitalWrite(ledh, HIGH);
Serial.println(" ");
Serial.println("Merah Mati");
Serial.println("Kungin Mati");
Serial.println("Hijau Menyala");
delay(2000);
}
