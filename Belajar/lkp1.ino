// Deklarasi array yang berisi semua pin LED
int leds[] = {16, 5, 4, 0, 2}; // D0, D1, D2, D3, D4
int jumlahLED = 5; // Jumlah LED

void setup() {
  // Mengatur semua pin LED sebagai OUTPUT
  for (int i = 0; i < jumlahLED; i++) {
    pinMode(leds[i], OUTPUT);
  }
}

void loop() {
  // Menyalakan dan mematikan LED satu per satu
  for (int i = 0; i < jumlahLED; i++) {
    digitalWrite(leds[i], HIGH); // Nyalakan LED ke-i
    delay(1000);                 // Tunggu 1 detik
    digitalWrite(leds[i], LOW);  // Matikan LED ke-i
    delay(1000);                 // Tunggu 1 detik
  }
}
