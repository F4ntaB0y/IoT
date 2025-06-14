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
bool sensorEnabled = false;  // Flag untuk mengaktifkan/nonaktifkan sensor jarak
unsigned int currentDistanceCm = 0; // Variabel global untuk menyimpan jarak sensor terbaru

// Flag untuk mode operasi
bool autonomousMode = false;
bool petMode = false;

// Membuat objek server web pada port 80
ESP8266WebServer server(80);

unsigned long previousMillis = 0;      // Untuk batas waktu koneksi WiFi
unsigned long lastSensorReadMillis = 0; // Untuk interval pembacaan sensor
const long sensorReadInterval = 100;    // Interval pembacaan sensor dalam ms (misal: 100ms)

// Konstanta untuk batas jarak mode otomatis (disesuaikan untuk sensor di belakang)
const int OBSTACLE_STOP_DISTANCE = 15; // Jarak (cm) di mana robot harus berhenti (mode otonom)
const int OBSTACLE_TURN_DISTANCE = 25; // Jarak (cm) di mana robot harus mulai mempertimbangkan belok (mode otonom)

// Konstanta untuk batas jarak mode PET
const int PET_MODE_APPROACH_MIN = 10; // Jarak minimum (cm) untuk mulai mendekat
const int PET_MODE_APPROACH_MAX = 15; // Jarak maksimum (cm) untuk mulai mendekat
const int PET_MODE_STOP_DISTANCE = 5; // Jarak (cm) di mana robot harus berhenti total (mode PET)

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
void PetMode();

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
      currentDistanceCm = MAX_DISTANCE_CM + 1; // Assume very far if 0 or out of range
    } else {
      currentDistanceCm = distance_cm; // Update global variable
    }

    // Pemicu Buzzer
    // Buzzer akan berbunyi jika sensor diaktifkan dan:
    // 1. Dalam mode manual/otonom DAN objek sangat dekat (<10cm)
    // 2. Dalam mode PET DAN objek dalam rentang PET (5-15cm)
    if (sensorEnabled) {
      if (!petMode && currentDistanceCm > 0 && currentDistanceCm < 10) { // Mode non-PET, objek sangat dekat
        Serial.print("Objek terdeteksi pada ");
        Serial.print(currentDistanceCm);
        BeepHorn();
      } else if (petMode && currentDistanceCm > 0 && currentDistanceCm <= PET_MODE_APPROACH_MAX) { // Mode PET, objek dalam rentang deteksi
        Serial.print("Objek terdeteksi pada ");
        Serial.print(currentDistanceCm);
        BeepHorn();
      }
    }
  }

  // --- Process Commands (Always active, including during auto/PET modes) ---
  if (server.hasArg("State")) {
    command = server.arg("State"); // Get the command value
    Serial.print("Perintah diterima: ");
    Serial.println(command); // Print the received command to Serial monitor

    // Perintah STOP ('S') dan Sensor OFF ('w') selalu diproses untuk mematikan sensor
    if (command == "S") {
      autonomousMode = false; // Matikan mode otonom
      petMode = false;        // Matikan mode PET
      Stop();                 // Hentikan motor
      Serial.println("STOP!");
    }
    else if (command == "w") { // Perintah 'w' untuk Sensor OFF
      sensorEnabled = false;
      TurnLightOff();
      Serial.println("Sensor jarak dinonaktifkan dan lampu mati!");
      // Jika sensor dimatikan secara manual, mode cerdas juga harus mati
      if (autonomousMode || petMode) {
        autonomousMode = false;
        petMode = false;
        Stop();
        Serial.println("Mode Otomatis juga dinonaktifkan saat sensor dimatikan.");
      }
    }
    // Perintah untuk mengaktifkan/menonaktifkan mode otomatis
    else if (command == "A") { // 'A' untuk Autonomous Mode ON
      autonomousMode = true;
      petMode = false;       // Pastikan mode PET nonaktif
      sensorEnabled = true;  // Aktifkan sensor saat mode otonom ON
      TurnLightOn();         // Nyalakan LED sebagai indikator
      Serial.println("Mode Otomatis AKTIF!");
      Backward(); // Mulai mundur saat mode otomatis aktif (sensor di belakang)
    }
    else if (command == "a") { // 'a' untuk Autonomous Mode OFF
      autonomousMode = false;
      Stop(); // Berhenti saat mode otomatis dinonaktifkan
      Serial.println("Mode Otomatis NONAKTIF!");
      // Sensor tetap pada status sebelumnya (tidak diubah oleh 'a')
    }
    // Perintah untuk mengaktifkan/menonaktifkan mode PET
    else if (command == "P") { // 'P' untuk Pet Mode ON
      petMode = true;
      autonomousMode = false; // Pastikan mode Otomatis nonaktif
      sensorEnabled = true;  // Aktifkan sensor saat mode PET ON
      TurnLightOn();         // Nyalakan LED sebagai indikator
      Serial.println("Mode PET AKTIF!");
      Stop(); // Mulai dengan berhenti dan menunggu objek
    }
    else if (command == "p") { // 'p' untuk Pet Mode OFF
      petMode = false;
      Stop(); // Berhenti saat mode PET dinonaktifkan
      Serial.println("Mode PET NONAKTIF!");
      // Sensor tetap pada status sebelumnya (tidak diubah oleh 'p')
    }
    // Perintah untuk mengaktifkan sensor ON (secara manual)
    else if (command == "W") { // 'W' untuk Sensor ON
      sensorEnabled = true;
      TurnLightOn();
      Serial.println("Sensor jarak diaktifkan dan lampu menyala!");
    }
    // Hanya memproses perintah manual motor lainnya jika TIDAK dalam mode otomatis/PET
    // DAN TIDAK MENGUBAH STATUS SENSORENABLED
    else if (!autonomousMode && !petMode) {
      if (command == "F") { Forward(); /* Sensor status remains unchanged */ }
      else if (command == "B") { Backward(); /* Sensor status remains unchanged */ }
      else if (command == "R") { TurnRight(); /* Sensor status remains unchanged */ }
      else if (command == "L") { TurnLeft(); /* Sensor status remains unchanged */ }
      else if (command == "V") { BeepHorn(); /* Sensor status remains unchanged */ }
    }
  }

  // --- Logika Mode Otomatis / PET ---
  if (autonomousMode) {
    AutonomousMode();
  } else if (petMode) {
    PetMode();
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
    overflow: hidden; /* Prevent scrolling */
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; /* Modern font */
    background-color: #e9ecef; /* Lighter background */
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center; /* Vertically center content */
    text-align: center;
  }
  .header {
    width: 90%;
    max-width: 800px; /* Increased max-width for desktop */
    background-color: #0056b3; /* Darker blue header */
    color: white;
    padding: 20px 0;
    border-radius: 12px; /* More rounded corners */
    margin-bottom: 25px;
    flex-shrink: 0;
    box-shadow: 0 6px 15px rgba(0,0,0,0.2); /* Stronger shadow */
  }
  .header h1 {
    margin: 0;
    font-size: 2em; /* Larger header font */
    letter-spacing: 1px;
  }
  .header p {
    margin: 8px 0 0;
    font-size: 1.1em;
    opacity: 0.9;
  }
  .distance-display {
    font-size: 2.2em; /* Larger distance font */
    font-weight: bold;
    color: #212529; /* Darker text */
    margin-bottom: 25px;
    padding: 18px 35px;
    background-color: #ffffff; /* White background */
    border-radius: 12px;
    box-shadow: 0 4px 10px rgba(0,0,0,0.1);
    flex-shrink: 0;
    min-width: 200px; /* Ensure it doesn't get too small */
  }
  .container {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 20px; /* Larger gap */
    width: 95%;
    max-width: 800px; /* Increased max-width for desktop */
    height: auto; /* Let content define height for flexibility */
    padding: 30px; /* Larger padding */
    background-color: #f8f9fa; /* Light grey background */
    border-radius: 12px;
    box-shadow: 0 8px 20px rgba(0,0,0,0.18); /* Stronger shadow */
    align-items: stretch;
    justify-items: stretch;
  }
  .button {
    width: 100%;
    height: auto; /* Adjust height dynamically */
    min-height: 80px; /* Minimum height for better touch target */
    padding: 15px 0;
    font-size: 2.5em; /* Larger button font */
    font-weight: bold;
    color: white;
    background-color: #28a745; /* Bootstrap green */
    border: none;
    border-radius: 10px;
    cursor: pointer;
    text-align: center;
    text-decoration: none;
    display: flex;
    justify-content: center;
    align-items: center;
    transition: background-color 0.2s ease, transform 0.1s ease; /* Smooth transitions */
    -webkit-tap-highlight-color: rgba(0,0,0,0);
  }
  .button:active {
    background-color: #218838; /* Darker on active */
    transform: translateY(2px); /* Slight press effect */
  }

  /* Specific button colors */
  .button.stop { background-color: #dc3545; } /* Bootstrap red */
  .button.stop:active { background-color: #c82333; }
  .button.horn { background-color: #ffc107; color: #343a40; } /* Bootstrap yellow, dark text */
  .button.horn:active { background-color: #e0a800; }
  .button.sensor { background-color: #6f42c1; } /* Bootstrap purple */
  .button.sensor:active { background-color: #5a2e9e; }
  .button.autonomous { background-color: #20c997; } /* Bootstrap teal */
  .button.autonomous:active { background-color: #19a47c; }
  .button.pet { background-color: #fd7e14; } /* Bootstrap orange */
  .button.pet:active { background-color: #e46001; }

  /* Directional buttons */
  .directional-button {
    background-color: #007bff; /* Bootstrap blue */
  }
  .directional-button:active {
    background-color: #0056b3;
  }
  .button.icon-button {
    font-size: 1.4em; /* Adjusted font size for multi-line text */
    padding: 12px;
    line-height: 1.3;
    min-height: 60px; /* Ensure smaller buttons are still touchable */
  }
  .button.stop-park {
    grid-column: span 3;
    font-size: 3.2em; /* Larger STOP button font */
    background-color: #6c757d; /* Bootstrap grey */
  }
  .button.stop-park:active {
    background-color: #5a6268;
  }
  .footer-text {
    margin-top: 25px;
    font-size: 1em;
    color: #6c757d; /* Muted grey for footer */
    flex-shrink: 0;
  }

  /* Media Queries for Responsiveness */
  /* For portrait mobile */
  @media (max-width: 576px) and (orientation: portrait) {
    .header {
      padding: 15px 0;
      margin-bottom: 15px;
    }
    .header h1 {
      font-size: 1.8em;
    }
    .header p {
      font-size: 1em;
    }
    .distance-display {
      font-size: 1.6em;
      margin-bottom: 15px;
      padding: 12px 25px;
    }
    .container {
      gap: 15px;
      padding: 20px;
      height: auto; /* Allow dynamic height based on content */
      max-height: none; /* Remove max-height constraint */
    }
    .button {
      font-size: 2em;
      min-height: 70px;
    }
    .button.icon-button {
      font-size: 1.1em;
      padding: 10px;
      min-height: 50px;
    }
    .button.stop-park {
      font-size: 2.5em;
    }
    .footer-text {
      margin-top: 15px;
      font-size: 0.9em;
    }
  }

  /* For landscape mobile/tablet */
  @media (max-height: 600px) and (orientation: landscape) {
    body {
      padding: 10px;
      justify-content: flex-start; /* Align content to top in landscape to save space */
    }
    .header {
      margin-bottom: 15px;
      padding: 15px 0;
    }
    .header h1 {
      font-size: 1.6em;
    }
    .header p {
      font-size: 0.9em;
    }
    .distance-display {
      font-size: 1.5em;
      margin-bottom: 15px;
      padding: 10px 20px;
    }
    .container {
      height: auto; /* Allow dynamic height */
      max-height: none; /* Remove fixed height constraint */
      gap: 10px;
      padding: 15px;
    }
    .button {
      font-size: 1.8em;
      min-height: 60px;
    }
    .button.icon-button {
      font-size: 1em;
      padding: 8px;
      min-height: 40px;
    }
    .button.stop-park {
      font-size: 2.2em;
    }
    .footer-text {
      margin-top: 10px;
      font-size: 0.8em;
    }
  }

  /* Further optimization for very wide screens (e.g., large monitors) */
  @media (min-width: 1024px) {
    .header, .distance-display, .container {
      padding-left: 40px;
      padding-right: 40px;
      max-width: 900px; /* Even wider for very large screens */
    }
    .button {
        min-height: 90px;
        font-size: 2.8em;
    }
    .button.icon-button {
        font-size: 1.6em;
        min-height: 70px;
    }
    .button.stop-park {
        font-size: 3.5em;
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
    // Use the same robot IP as this page
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

  // Call getDistance every 500ms for real-time updates
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
      <button class="button icon-button sensor" onclick="sendCommand('W')">Lampu ON</button>
      <button class="button icon-button sensor" onclick="sendCommand('w')">Lampu OFF</button>
      <button class="button icon-button autonomous" onclick="sendCommand('A')">MODE OTOMATIS ON</button>
      <button class="button icon-button autonomous" onclick="sendCommand('a')">MODE OTOMATIS OFF</button>
      <button class="button icon-button pet" onclick="sendCommand('P')">MODE PELIHARAAN ON</button>
      <button class="button icon-button pet" onclick="sendCommand('p')">MODE PELIHARAAN OFF</button>
    </div>
    <button class="button directional-button" ontouchstart="sendCommand('R')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8594;</button>

    <div></div>
    <button class="button directional-button" ontouchstart="sendCommand('B')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8595;</button>
    <div></div>
    
    <button class="button stop-park" onclick="sendCommand('S')">STOP</button>
  </div>

  <p class="footer-text">Kontrol robot Anda melalui WiFi. Gunakan tombol panah untuk arah, dan tombol di tengah untuk fungsi tambahan.</p>

</body>
</html>
)rawliteral";

  // Get current IP address to display on the page
  String ipAddress;
  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
  } else {
    ipAddress = WiFi.softAPIP().toString();
  }
  html.replace("%IP_ADDRESS%", ipAddress); // Replace placeholder with actual IP
  html.replace("%MAX_DISTANCE_CM%", String(MAX_DISTANCE_CM)); // Replace placeholder for far distance display

  server.send(200, "text/html", html);
}

// Function to handle unknown URI requests (HTTP 404 Not Found)
void handleNotFound() {
  server.send(404, "text/plain", "404: Tidak Ditemukan");
}

// --- Function to handle requests for sensor distance ---
void handleGetDistance() {
  server.send(200, "text/plain", String(currentDistanceCm));
}

// --- Motor Control Functions ---

// Move both motors forward
void Forward() {
  // Left Motor (Motor A)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Right Motor (Motor B)
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Move both motors backward
void Backward() {
  // Left Motor (Motor A)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Right Motor (Motor B)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Turn robot right in place (Left motor forward, Right motor backward)
void TurnRight() {
  // Left Motor (Motor A) - Forward
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Right Motor (Motor B) - Backward
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Turn robot left in place (Left motor backward, Right motor forward)
void TurnLeft() {
  // Left Motor (Motor A) - Backward
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Right Motor (Motor B) - Forward
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Stop both motors immediately
void Stop() {
  // Left Motor (Motor A)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);

  // Right Motor (Motor B)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
}

// Activate buzzer (horn) for a short duration
void BeepHorn() {
  digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
  delay(150);                     // Beep for 150ms (this will momentarily block other code execution)
  digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
  delay(80);                      // Short pause (this will momentarily block other code execution)
}

// Function to turn on additional LED (connected to D2)
void TurnLightOn() {
  digitalWrite(LED_PIN, HIGH);
}

// Function to turn off additional LED (connected to D2)
void TurnLightOff() {
  digitalWrite(LED_PIN, LOW);
}

// Function for Autonomous Mode (adjusted for sensor at the back)
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

// Function for PET Mode (Pet) - Corrected to move backward and use buzzer
void PetMode() {
  sensorEnabled = true; // Pastikan sensor aktif di mode ini

  if (currentDistanceCm > PET_MODE_STOP_DISTANCE && currentDistanceCm <= PET_MODE_APPROACH_MAX) {
    // Objek terdeteksi dalam rentang mendekat (misal 5-15cm)
    Serial.print("Mode PET: Objek pada ");
    Serial.print(currentDistanceCm);
    Serial.println(" cm. Mendekat (Mundur)...");
    Backward(); // **Bergerak mundur** untuk mendekati objek (sensor di belakang)
    // Buzzer otomatis akan berbunyi karena kondisi di loop() sekarang mencakup mode PET
  } else if (currentDistanceCm <= PET_MODE_STOP_DISTANCE && currentDistanceCm != 0) {
    // Objek terlalu dekat (misal <= 5cm), berhenti total
    Serial.print("Mode PET: Objek terlalu dekat pada ");
    Serial.print(currentDistanceCm);
    Serial.println(" cm! BERHENTI.");
    Stop();
    // Buzzer otomatis akan berbunyi karena kondisi di loop() sekarang mencakup mode PET
  } else {
    // Objek di luar jangkauan deteksi (lebih dari PET_MODE_APPROACH_MAX atau tidak terdeteksi)
    Serial.println("Mode PET: Tidak ada objek dalam jangkauan. Diam.");
    Stop(); // Berhenti dan tunggu objek masuk ke jangkauan
  }
}
