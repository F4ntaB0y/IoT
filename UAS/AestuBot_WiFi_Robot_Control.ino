/*******************************************************************************
 * FILE: AestuBot_WiFi_Robot_Control.ino
 * DESKRIPSI: Kode untuk mengontrol mobil RC 2 motor menggunakan ESP8266,
 * driver motor L298N, server web WiFi, dan sensor jarak HC-SR04, juga LED.
 * Memungkinkan kontrol arah (maju, mundur, belok, berhenti) pada kecepatan penuh,
 * kontrol manual untuk mengaktifkan/menonaktifkan deteksi jarak sensor (memicu
 * buzzer jika objek di bawah 10 cm), dan kontrol LED.
 * AUTHOR: Muh. Arya Pangestu
 * TANGGAL: 14 Juni 2025
 *******************************************************************************/

#include <ESP8266WiFi.h>      // Library untuk fungsionalitas WiFi ESP8266
#include <ESP8266WebServer.h> // Library untuk membuat server web di ESP8266
#include <ArduinoOTA.h>       // Library untuk pembaruan firmware Over-The-Air (OTA)
#include <NewPing.h>          // Library untuk sensor jarak ultrasonik HC-SR04

// --- Motor 1 (Roda Kiri) - Terhubung ke OUT1 & OUT2 L298N ---
const int IN1_LEFT = D3;    // Pin IN1 L298N untuk kontrol arah motor kiri
const int IN2_LEFT = D4;    // Pin IN2 L298N untuk kontrol arah motor kiri

// --- Motor 2 (Roda Kanan) - Terhubung ke OUT3 & OUT4 L298N ---
const int IN1_RIGHT = D6;   // Pin IN3 L298N untuk kontrol arah motor kanan
const int IN2_RIGHT = D7;   // Pin IN4 L298N untuk kontrol arah motor kanan

// --- Pin Komponen Robot Lainnya ---
const int BUZZER_PIN = D5;  // Pin digital untuk buzzer aktif (misalnya, D5/GPIO14)
const int LED_PIN = D2;     // Pin digital D2 (GPIO4) untuk LED tambahan

// --- Definisi Pin dan Objek untuk Sensor HC-SR04 ---
#define HCSR04_TRIGGER_PIN D8     // Pin Trigger HC-SR04
#define HCSR04_ECHO_PIN    D0     // Pin Echo HC-SR04
#define MAX_DISTANCE_CM    100    // Jarak maksimum yang ingin dideteksi (dalam centimeter)

// Membuat objek NewPing untuk sensor HC-SR04
NewPing sonar(HCSR04_TRIGGER_PIN, HCSR04_ECHO_PIN, MAX_DISTANCE_CM);

// --- Variabel Global ---
String command;              // String untuk menyimpan perintah dari aplikasi web
int MOTOR_SPEED = 1023;      // Kecepatan motor default, selalu diatur ke nilai maksimum (0-1023 untuk analogWrite)
int speed_Coeff = 3;         // Koefisien untuk kecepatan belok diferensial (memperlambat salah satu motor)
bool sensorEnabled = false;  // Flag untuk mengaktifkan/menonaktifkan sensor jarak
unsigned int currentDistanceCm = 0; // Variabel global untuk menyimpan jarak sensor terbaru

// Flag untuk mode otomatis
bool autonomousMode = false;

// Membuat objek server web pada port 80
ESP8266WebServer server(80);

unsigned long previousMillis = 0;      // Untuk batas waktu koneksi WiFi
unsigned long lastSensorReadMillis = 0; // Untuk interval pembacaan sensor
const long sensorReadInterval = 100;    // Interval pembacaan sensor dalam ms (misal: 100ms)

// Konstanta untuk batas jarak mode otomatis (disesuaikan untuk sensor di belakang)
const int OBSTACLE_STOP_DISTANCE = 15; // Jarak (cm) di mana robot harus berhenti
const int OBSTACLE_TURN_DISTANCE = 25; // Jarak (cm) di mana robot harus mulai mempertimbangkan belok

// --- Konfigurasi WiFi ---
// Jika dibiarkan kosong, ESP8266 akan mencoba terhubung selama 10 detik,
// lalu kembali ke mode AP (membuat jaringan WiFi-nya sendiri).
String sta_ssid = "tesESP";         // Nama jaringan WiFi Anda (SSID)
String sta_password = "123456789";  // Kata sandi jaringan WiFi Anda

// --- Prototipe Fungsi (untuk organisasi kode yang lebih baik) ---
void HTTP_handleRoot();
void handleNotFound();
void handleGetDistance();
void Forward();
void Backward();
void TurnRight();
void TurnLeft();
void ForwardLeft();
void BackwardLeft();
void ForwardRight();
void BackwardRight();
void Stop();
void BeepHorn();
void TurnLightOn();
void TurnLightOff();
void AutonomousMode();

void setup() {
  Serial.begin(115200); // Mengatur komunikasi Serial untuk debugging
  Serial.println();
  Serial.println("=============================================");
  Serial.println("       Mode Kontrol Robot WiFi Jarak Jauh    ");
  Serial.println("   (Kecepatan Penuh dengan Sensor Jarak & LED)");
  Serial.println("=============================================");

  // --- Inisialisasi Pin Komponen Robot ---
  pinMode(BUZZER_PIN, OUTPUT);    // Pin buzzer sebagai output
  digitalWrite(BUZZER_PIN, LOW);  // Pastikan buzzer mati di awal

  pinMode(LED_PIN, OUTPUT);       // Atur pin D2 sebagai output untuk LED
  digitalWrite(LED_PIN, LOW);     // Pastikan LED mati di awal

  // Mengatur semua pin kontrol motor sebagai output
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);

  // Pastikan motor berhenti saat startup
  Stop(); // Memanggil fungsi Stop untuk mengatur semua pin motor ke LOW

  // --- Pengaturan WiFi ---
  // Mengatur hostname WiFi NodeMCU berdasarkan alamat MAC chip (untuk identifikasi lebih mudah)
  String chip_id = String(ESP.getChipId(), HEX);
  int i = chip_id.length() - 4;
  chip_id = chip_id.substring(i);
  chip_id = "AestuBot" + chip_id; // Contoh: EstuBotABCD
  String hostname(chip_id);

  Serial.println();
  Serial.print("Hostname: ");
  Serial.println(hostname);

  // Mencoba terhubung ke jaringan WiFi yang dikenal (mode STA)
  WiFi.mode(WIFI_STA);
  // Hanya mencoba terhubung jika SSID disediakan
  if (sta_ssid.length() > 0) {
    WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
    Serial.println("");
    Serial.print("Menghubungkan ke: ");
    Serial.println(sta_ssid);
    Serial.print("Kata Sandi: ");
    Serial.println(sta_password);

    // Menunggu koneksi WiFi, dengan batas waktu
    unsigned long currentMillis = millis();
    previousMillis = currentMillis;
    while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= 10000) {
      delay(500);
      Serial.print(".");
      currentMillis = millis();
    }
  }

  // Memeriksa status koneksi dan bertindak sesuai (mode STA atau AP)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi Terhubung! (Mode STA)");
    Serial.print("Alamat IP: ");
    Serial.println(WiFi.localIP());
    delay(1000);
  } else {
    Serial.println("");
    Serial.println("Koneksi WiFi gagal atau kredensial tidak disediakan.");
    Serial.println("Kembali ke Mode Access Point (AP).");
    WiFi.mode(WIFI_AP);       // Mengatur ESP8266 sebagai Access Point
    WiFi.softAP(hostname.c_str()); // Membuat jaringan WiFi dengan hostname sebagai SSID
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("Alamat IP AP: ");
    Serial.println(myIP);
    delay(1000);
  }

  // --- Pengaturan Server Web ---
  server.on("/", HTTP_handleRoot);   // Menangani permintaan ke URL root
  server.on("/distance", handleGetDistance); // Handler for sensor distance
  server.onNotFound(handleNotFound); // Menangani URL yang tidak dikenal (404)
  server.begin();                    // Memulai server web
  Serial.println("Server HTTP dimulai");

  // --- Pengaturan OTA (Over-The-Air) Updates ---
  // Memungkinkan mengunggah firmware baru secara nirkabel melalui Arduino IDE.
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.begin();
  Serial.println("OTA diaktifkan.");
}

void loop() {
  ArduinoOTA.handle();       // Mendengarkan permintaan pembaruan OTA
  server.handleClient();     // Mendengarkan permintaan HTTP yang masuk dari klien

  // --- Pembacaan Sensor Jarak ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorReadMillis >= sensorReadInterval) {
    lastSensorReadMillis = currentMillis; // Perbarui waktu terakhir pembacaan

    unsigned int distance_cm = sonar.ping_cm(); // Membaca jarak dalam cm
    // NewPing mengembalikan 0 jika tidak ada echo (objek di luar jangkauan MAX_DISTANCE_CM)
    // atau di bawah min_cm (default 2cm).
    // Jika 0, kita bisa set ke MAX_DISTANCE_CM + 1 untuk menandakan "tidak ada objek" dalam jangkauan
    if (distance_cm == 0) {
      currentDistanceCm = MAX_DISTANCE_CM + 1; // Anggap sangat jauh jika 0 atau di luar jangkauan
    } else {
      currentDistanceCm = distance_cm; // Update global variable
    }

    // Pemicu Buzzer jika sensor diaktifkan dan ada objek sangat dekat
    if (sensorEnabled && currentDistanceCm > 0 && currentDistanceCm < 10) {
      Serial.print("Objek terdeteksi pada ");
      Serial.print(currentDistanceCm);
      Serial.println(" cm! Buzzer berbunyi!");
      BeepHorn(); // Memicu buzzer jika jarak kurang dari 10 cm
    }
  }

  // --- Pemrosesan Perintah (Selalu aktif, termasuk saat mode otomatis) ---
  if (server.hasArg("State")) {
    command = server.arg("State"); // Mendapatkan nilai perintah
    Serial.print("Perintah diterima: ");
    Serial.println(command); // Mencetak perintah yang diterima ke monitor Serial

    // Perintah STOP (S) selalu diproses terlebih dahulu untuk keamanan
    if (command == "S") {
      autonomousMode = false; // Pastikan mode otomatis dimatikan jika STOP ditekan
      Stop();
      Serial.println("STOP! Mode Otomatis dinonaktifkan.");
    }
    // Perintah untuk mengaktifkan/menonaktifkan mode otomatis
    else if (command == "A") { // 'A' untuk Autonomous Mode ON
      autonomousMode = true;
      sensorEnabled = true; // Pastikan sensor aktif di mode otomatis
      TurnLightOn(); // Nyalakan lampu LED sebagai indikator
      Serial.println("Mode Otomatis AKTIF!");
      // Mulai mundur saat mode otomatis aktif (karena sensor di belakang)
      Backward();
    }
    else if (command == "a") { // 'a' untuk Autonomous Mode OFF
      autonomousMode = false;
      Stop(); // Berhenti saat mode otomatis dinonaktifkan
      Serial.println("Mode Otomatis NONAKTIF!");
    }
    // Hanya memproses perintah manual motor lainnya jika TIDAK dalam mode otomatis
    else if (!autonomousMode) {
      if (command == "F") Forward();         // Bergerak maju (kecepatan penuh)
      else if (command == "B") Backward();   // Bergerak mundur (kecepatan penuh)
      else if (command == "R") TurnRight();  // Berbelok ke kanan di tempat (kecepatan penuh)
      else if (command == "L") TurnLeft();   // Berbelok ke kiri di tempat (kecepatan penuh)
      else if (command == "V") BeepHorn();         // Mengaktifkan buzzer (klakson) - ini adalah perintah manual
      else if (command == "W") {                 // Perintah untuk MENGAKTIFKAN sensor DAN MENYALAKAN lampu
        sensorEnabled = true;
        TurnLightOn(); // Nyalakan lampu LED
        Serial.println("Sensor jarak diaktifkan dan lampu menyala!");
      }
      else if (command == "w") {                 // Perintah untuk MENONAKTIFKAN sensor DAN MEMATIKAN lampu
        sensorEnabled = false;
        TurnLightOff(); // Matikan lampu LED
        Serial.println("Sensor jarak dinonaktifkan dan lampu mati!");
      }
    }
  }

  // --- Logika Mode Otomatis ---
  if (autonomousMode) {
    AutonomousMode();
  }
}

// --- HTTP Request Handlers ---

// Fungsi untuk menangani permintaan ke URL root "/"
void HTTP_handleRoot(void) {
  // Menggunakan R"rawliteral(...)rawliteral" untuk string HTML multi-baris
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Kontrol Robot WiFi</title>
<style>
  html, body {
    height: 100%;
    margin: 0;
    padding: 0;
    overflow: hidden; /* Mencegah scrolling */
    font-family: Arial, sans-serif;
    background-color: #f0f0f0;
  }
  body {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center; /* Memusatkan konten secara vertikal */
    text-align: center;
  }
  .header {
    width: 100%;
    max-width: 600px;
    background-color: #007bff;
    color: white;
    padding: 10px 0;
    border-radius: 8px;
    margin-bottom: 15px;
    flex-shrink: 0; /* Mencegah header menyusut */
  }
  .header h1 {
    margin: 0;
    font-size: 1.5em;
  }
  .header p {
    margin: 5px 0 0;
    font-size: 0.9em;
  }
  .distance-display {
    font-size: 1.8em;
    font-weight: bold;
    color: #333;
    margin-bottom: 15px;
    padding: 10px 20px;
    background-color: #fff;
    border-radius: 8px;
    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    flex-shrink: 0;
  }
  .container {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 15px;
    width: 95%; /* Menggunakan persentase agar responsif */
    max-width: 600px;
    height: 60vh; /* Menggunakan persentase tinggi viewport */
    max-height: 500px;
    background-color: white;
    padding: 20px;
    border-radius: 8px;
    box-shadow: 0 4px 8px rgba(0,0,0,0.1);
    align-items: stretch; /* Memastikan item meregang */
    justify-items: stretch; /* Memastikan item meregang */
  }
  .button {
    width: 100%;
    height: 100%; /* Tombol mengisi sel grid sepenuhnya */
    padding: 15px 0;
    font-size: 2.2em;
    font-weight: bold;
    color: white;
    background-color: #4CAF50; /* Hijau */
    border: none;
    border-radius: 8px;
    cursor: pointer;
    text-align: center;
    text-decoration: none;
    display: flex; /* Menggunakan flex untuk memusatkan konten tombol */
    justify-content: center;
    align-items: center;
    transition: background-color 0.3s ease;
    -webkit-tap-highlight-color: rgba(0,0,0,0); /* Mencegah highlight biru saat disentuh di mobile */
  }
  .button:active {
    background-color: #3e8e41;
  }

  /* Warna khusus untuk tombol aksi */
  .button.stop { background-color: #f44336; } /* Merah */
  .button.stop:active { background-color: #da190b; }
  .button.horn { background-color: #ff9800; } /* Oranye */
  .button.horn:active { background-color: #e68a00; }
  .button.light { background-color: #2196F3; } /* Biru */
  .button.light:active { background-color: #0b7dda; }
  .button.sensor { background-color: #9c27b0; } /* Ungu */
  .button.sensor:active { background-color: #7b1fa2; }
  .button.autonomous { background-color: #8BC34A; } /* Hijau muda untuk autonomous */
  .button.autonomous:active { background-color: #689F38; }

  /* Penyesuaian tata letak grid */
  .directional-button {
    background-color: #008CBA;
  }
  .directional-button:active {
    background-color: #005f7c;
  }
  .button.icon-button {
    font-size: 1.2em; /* Ukuran font disesuaikan */
    padding: 10px;
    line-height: 1.2; /* Menambahkan line-height untuk perataan teks */
  }
  .button.stop-park {
    grid-column: span 3;
    font-size: 2.5em;
    background-color: #607d8b;
  }
  .button.stop-park:active {
    background-color: #455a64;
  }
  .footer-text {
    margin-top: 15px;
    font-size: 0.8em;
    color: #555;
    flex-shrink: 0; /* Mencegah footer menyusut */
  }

  /* Untuk tampilan mobile agar lebih proporsional */
  @media (max-width: 480px) and (orientation: portrait) {
    .container {
      grid-template-columns: repeat(3, 1fr);
      height: 50vh; /* Sesuaikan tinggi untuk potret */
      gap: 10px;
      padding: 15px;
    }
    .button {
      font-size: 1.8em;
    }
    .button.icon-button {
      font-size: 1em;
    }
    .button.stop-park {
        font-size: 2em;
    }
  }
  
  @media (max-height: 480px) and (orientation: landscape) {
    .header {
        margin-bottom: 5px;
    }
    .header h1 {
        font-size: 1.2em;
    }
    .container {
        height: 70vh;
        gap: 8px;
        padding: 10px;
    }
    .button {
        font-size: 1.5em;
    }
    .footer-text {
        margin-top: 5px;
        font-size: 0.7em;
    }
  }
</style>
<script>
  function sendCommand(cmd) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        // console.log("Command '" + cmd + "' sent. Response: " + this.responseText);
      }
    };
    // Menggunakan IP robot yang sama dengan halaman ini
    xhttp.open("GET", "/?State=" + cmd, true);
    xhttp.send();
  }

  function getDistance() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        let distance = this.responseText;
        if (distance == %MAX_DISTANCE_CM% + 1) { // NewPing returns 0 for out of range or too close, then mapped to MAX_DISTANCE_CM + 1
          document.getElementById("distanceValue").innerText = " > " + %MAX_DISTANCE_CM% + " (Jauh)";
        } else {
          document.getElementById("distanceValue").innerText = distance;
        }
      }
    };
    xhttp.open("GET", "/distance", true);
    xhttp.send();
  }

  // Panggil getDistance setiap 500ms untuk pembaruan real-time
  setInterval(getDistance, 500);
</script>
</head>
<body>

  <div class="header">
    <h1>ESP8266 WiFi Robot Car</h1>
    <p>IP: %IP_ADDRESS%</p>
  </div>

  <div class="distance-display">
    Jarak: <span id="distanceValue">N/A</span> cm
  </div>

  <div class="container">
    <div></div>
    <button class="button directional-button" ontouchstart="sendCommand('F')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8593;</button>
    <div></div>

    <button class="button directional-button" ontouchstart="sendCommand('L')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8592;</button>
    <div style="display: flex; flex-direction: column; gap: 10px;">
      <button class="button icon-button horn" ontouchstart="sendCommand('V')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#128266; Klakson</button>
      <button class="button icon-button sensor" onclick="sendCommand('W')">Sensor ON</button>
      <button class="button icon-button sensor" onclick="sendCommand('w')">Sensor OFF</button>
      <button class="button icon-button autonomous" onclick="sendCommand('A')">MODE OTOMATIS ON</button>
      <button class="button icon-button autonomous" onclick="sendCommand('a')">MODE OTOMATIS OFF</button>
    </div>
    <button class="button directional-button" ontouchstart="sendCommand('R')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8594;</button>

    <div></div>
    <button class="button directional-button" ontouchstart="sendCommand('B')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8595;</button>
    <div></div>
    
    <button class="button stop-park" onclick="sendCommand('S')">STOP</button>
  </div>

</body>
</html>
)rawliteral";

  // Dapatkan alamat IP saat ini untuk ditampilkan di halaman
  String ipAddress;
  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
  } else {
    ipAddress = WiFi.softAPIP().toString();
  }
  html.replace("%IP_ADDRESS%", ipAddress); // Ganti placeholder dengan IP aktual
  html.replace("%MAX_DISTANCE_CM%", String(MAX_DISTANCE_CM)); // Ganti placeholder untuk tampilan jarak jauh

  server.send(200, "text/html", html);
}

// Fungsi untuk menangani permintaan URI yang tidak dikenal (HTTP 404 Not Found)
void handleNotFound() {
  server.send(404, "text/plain", "404: Tidak Ditemukan");
}

// --- Function to handle requests for sensor distance ---
void handleGetDistance() {
  server.send(200, "text/plain", String(currentDistanceCm));
}

// --- Fungsi Kontrol Motor ---

// Menggerakkan kedua motor maju
void Forward() {
  // Motor Kiri (Motor A)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B)
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Menggerakkan kedua motor mundur
void Backward() {
  // Motor Kiri (Motor A)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Motor Kanan (Motor B)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Membelokkan robot ke kanan di tempat (Motor kiri maju, Motor kanan mundur)
void TurnRight() {
  // Motor Kiri (Motor A) - Maju
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B) - Mundur
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Membelokkan robot ke kiri di tempat (Motor kiri mundur, Motor kanan maju)
void TurnLeft() {
  // Motor Kiri (Motor A) - Mundur
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Motor Kanan (Motor B) - Maju
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Menghentikan kedua motor segera
void Stop() {
  // Motor Kiri (Motor A)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
}

// Mengaktifkan buzzer (klakson) untuk durasi singkat
void BeepHorn() {
  digitalWrite(BUZZER_PIN, HIGH); // Menyalakan buzzer
  delay(150);                     // Berbunyi selama 150ms (ini akan memblokir eksekusi kode lain sejenak)
  digitalWrite(BUZZER_PIN, LOW);  // Mematikan buzzer
  delay(80);                      // Jeda singkat (ini akan memblokir eksekusi kode lain sejenak)
}

// Fungsi untuk menyalakan LED tambahan (terhubung ke D2)
void TurnLightOn() {
  digitalWrite(LED_PIN, HIGH);
}

// Fungsi untuk mematikan LED tambahan (terhubung ke D2)
void TurnLightOff() {
  digitalWrite(LED_PIN, LOW);
}

// Fungsi untuk Mode Otomatis (disesuaikan untuk sensor di belakang)
void AutonomousMode() {
  // Pastikan sensor diaktifkan di mode ini
  sensorEnabled = true;

  // Logika penghindaran halangan dasar (robot bergerak mundur, sensor di belakang)
  if (currentDistanceCm <= OBSTACLE_STOP_DISTANCE && currentDistanceCm != 0) { // Objek sangat dekat di belakang
    Serial.println("Mode Otomatis: Objek sangat dekat di belakang! Berhenti & Maju...");
    Stop();
    delay(200); // Berhenti sebentar
    Forward(); // **Maju** untuk menghindari halangan di belakang
    delay(500); // Maju sebentar
    Stop();
    delay(100); // Berhenti lagi
    // Kemudian putar secara acak untuk mencari jalur lain
    if (random(2) == 0) { // Pilih acak kiri atau kanan
      TurnLeft();
      Serial.println("Mode Otomatis: Belok Kiri.");
    } else {
      TurnRight();
      Serial.println("Mode Otomatis: Belok Kanan.");
    }
    delay(500); // Belok sebentar
    Stop();
    delay(100); // Berhenti sebelum melanjutkan
  } else if (currentDistanceCm > OBSTACLE_STOP_DISTANCE && currentDistanceCm <= OBSTACLE_TURN_DISTANCE && currentDistanceCm != 0) { // Objek agak dekat di belakang, coba belok
    Serial.println("Mode Otomatis: Objek agak dekat di belakang! Sesuaikan arah...");
    // Coba belok ringan ke salah satu sisi
    if (random(2) == 0) {
      TurnLeft();
    } else {
      TurnRight();
    }
    delay(300); // Belok sebentar
    Stop();
    delay(50);
  } else { // Tidak ada objek di belakang dalam jarak aman, terus mundur
    Serial.println("Mode Otomatis: Bergerak Mundur.");
    Backward(); // Robot terus mundur (gerakan dasar mode otomatis)
  }
}
