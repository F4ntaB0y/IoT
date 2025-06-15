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
const int BUZZER_PIN = D5;  // Pin digital untuk buzzer aktif dan LED Peringatan
const int LED_PIN = D2;     // Pin digital D2 (GPIO4) untuk LED tambahan

// --- Definisi Pin dan Objek untuk Sensor HC-SR04 ---
#define HCSR04_TRIGGER_PIN D8       // Pin Trigger HC-SR04 (mengirim gelombang suara)
#define HCSR04_ECHO_PIN    D0       // Pin Echo HC-SR04 (menerima gelombang pantul)
#define MAX_DISTANCE_CM    100      // Jarak maksimum yang ingin dideteksi (dalam centimeter)

// Membuat objek NewPing untuk sensor HC-SR04.
// Parameter: pin trigger, pin echo, dan jarak maksimum yang dapat diukur.
NewPing sonar(HCSR04_TRIGGER_PIN, HCSR04_ECHO_PIN, MAX_DISTANCE_CM);

// --- Variabel Global ---
String command;             // String untuk menyimpan perintah yang diterima dari aplikasi web.
int MOTOR_SPEED = 1023;     // Kecepatan motor default, selalu diatur ke nilai maksimum (0-1023 untuk analogWrite).
                            // Karena L298N dikontrol dengan digitalWrite (HIGH/LOW), nilai ini tidak digunakan untuk kecepatan.
int speed_Coeff = 3;        // Koefisien untuk kecepatan belok diferensial (memperlambat salah satu motor).
                            // Variabel ini tidak digunakan dalam implementasi saat ini karena motor selalu pada kecepatan penuh.
bool sensorEnabled = false; // Flag untuk mengaktifkan/nonaktifkan pembacaan sensor jarak dan pemicu buzzer.
unsigned int currentDistanceCm = 0; // Variabel global untuk menyimpan jarak sensor terbaru yang terbaca.

// Flag untuk mode operasi robot
bool autonomousMode = false; // Jika true, robot beroperasi dalam mode penghindaran halangan.
bool petMode = false;        // Jika true, robot beroperasi dalam mode "mengikuti/mendekati objek".

// Membuat objek server web pada port 80 (port HTTP standar).
ESP8266WebServer server(80);

unsigned long previousMillis = 0;     // Variabel untuk melacak waktu dan mengimplementasikan batas waktu koneksi WiFi.
unsigned long lastSensorReadMillis = 0; // Variabel untuk melacak waktu pembacaan sensor terakhir.
const long sensorReadInterval = 100;    // Interval (dalam milidetik) antara pembacaan sensor (misal: 100ms).

// Konstanta untuk batas jarak mode otonom (disesuaikan untuk sensor di belakang robot).
const int OBSTACLE_STOP_DISTANCE = 15; // Jarak (cm) di mana robot harus berhenti total (mode otonom).
const int OBSTACLE_TURN_DISTANCE = 25; // Jarak (cm) di mana robot harus mulai mempertimbangkan belok (mode otonom).

// Konstanta untuk batas jarak mode PET.
const int PET_MODE_APPROACH_MIN = 10; // Jarak minimum (cm) untuk mulai mendekat objek.
const int PET_MODE_APPROACH_MAX = 15; // Jarak maksimum (cm) untuk mulai mendekat objek.
const int PET_MODE_STOP_DISTANCE = 5; // Jarak (cm) di mana robot harus berhenti total karena objek terlalu dekat (mode PET).

// --- Konfigurasi WiFi ---
// Jika `sta_ssid` dibiarkan kosong, ESP8266 akan mencoba terhubung sebagai STA selama 10 detik,
// lalu kembali ke mode AP (membuat jaringan WiFi-nya sendiri).
String sta_ssid = "tesESP";         // Nama jaringan WiFi Anda (SSID) yang ingin dihubungkan.
String sta_password = "123456789";  // Kata sandi jaringan WiFi Anda.

// --- Prototipe Fungsi (untuk organisasi kode yang lebih baik dan deklarasi maju) ---
void HTTP_handleRoot();      // Menangani permintaan HTTP ke URL root ("/")
void handleNotFound();       // Menangani permintaan HTTP untuk URL yang tidak ditemukan (404)
void handleGetDistance();    // Menangani permintaan HTTP untuk mendapatkan data jarak sensor
void Forward();              // Menggerakkan robot maju
void Backward();             // Menggerakkan robot mundur
void TurnRight();            // Membuat robot berbelok ke kanan di tempat
void TurnLeft();             // Membuat robot berbelok ke kiri di tempat
// Fungsi-fungsi berikut (ForwardLeft, BackwardLeft, ForwardRight, BackwardRight)
// tidak diimplementasikan atau digunakan dalam kode yang diberikan,
// namun prototipenya ada. Mereka akan digunakan untuk belok dengan diferensial.
void ForwardLeft();          // (Tidak digunakan) Menggerakkan maju dan sedikit ke kiri
void BackwardLeft();         // (Tidak digunakan) Menggerakkan mundur dan sedikit ke kiri
void ForwardRight();         // (Tidak digunakan) Menggerakkan maju dan sedikit ke kanan
void BackwardRight();        // (Tidak digunakan) Menggerakkan mundur dan sedikit ke kanan
void Stop();                 // Menghentikan semua motor
void BeepHorn();             // Mengaktifkan buzzer (klakson) sebentar
void TurnLightOn();          // Menyalakan LED tambahan
void TurnLightOff();         // Mematikan LED tambahan
void AutonomousMode();       // Logika untuk mode robot otonom (penghindaran halangan)
void PetMode();              // Logika untuk mode "Peliharaan" (mendekati objek)

void setup() {
  Serial.begin(115200); // Mengatur komunikasi Serial untuk debugging pada baud rate 115200.
  Serial.println();
  Serial.println("=============================================");
  Serial.println("         Mode Kontrol Robot WiFi Jarak Jauh    ");
  Serial.println("   (Kecepatan Penuh dengan Sensor Jarak & LED)");
  Serial.println("=============================================");

  // --- Inisialisasi Pin Komponen Robot ---
  pinMode(BUZZER_PIN, OUTPUT);    // Mengatur pin buzzer sebagai output.
  digitalWrite(BUZZER_PIN, LOW);  // Memastikan buzzer mati di awal.

  pinMode(LED_PIN, OUTPUT);       // Mengatur pin D2 (GPIO4) sebagai output untuk LED.
  digitalWrite(LED_PIN, LOW);     // Memastikan LED mati di awal.

  // Mengatur semua pin kontrol motor sebagai output.
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);

  // --- Pengaturan WiFi ---
  // Mengatur hostname WiFi NodeMCU berdasarkan alamat MAC chip (untuk identifikasi lebih mudah).
  String chip_id = String(ESP.getChipId(), HEX); // Mendapatkan ID chip sebagai string heksadesimal.
  int i = chip_id.length() - 4;                  // Mengambil 4 karakter terakhir dari ID chip.
  chip_id = chip_id.substring(i);
  chip_id = "AestuBot" + chip_id; // Contoh hostname: AestuBotABCD
  String hostname(chip_id);       // Mengubah string `chip_id` menjadi `String` `hostname`.

  Serial.println();
  Serial.print("Hostname: ");
  Serial.println(hostname);

  // Mencoba terhubung ke jaringan WiFi yang dikenal (mode STA - Station).
  WiFi.mode(WIFI_STA); // Mengatur ESP8266 dalam mode Station.
  // Hanya mencoba terhubung jika SSID disediakan (tidak kosong).
  if (sta_ssid.length() > 0) {
    WiFi.begin(sta_ssid.c_str(), sta_password.c_str()); // Memulai koneksi WiFi.
    Serial.println("");
    Serial.print("Menghubungkan ke: ");
    Serial.println(sta_ssid);
    Serial.print("Kata Sandi: ");
    Serial.println(sta_password);

    // Menunggu koneksi WiFi, dengan batas waktu 10 detik.
    unsigned long currentMillis = millis();
    previousMillis = currentMillis;
    while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= 10000) {
      delay(500);       // Menunggu 500ms.
      Serial.print("."); // Mencetak titik untuk menunjukkan progress.
      currentMillis = millis(); // Memperbarui waktu saat ini.
    }
  }

  // Memeriksa status koneksi dan bertindak sesuai (mode STA atau AP).
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi Terhubung! (Mode STA)");
    Serial.print("Alamat IP: ");
    Serial.println(WiFi.localIP()); // Mencetak alamat IP yang didapat dari router.
    delay(1000);
  } else {
    Serial.println("");
    Serial.println("Koneksi WiFi gagal atau kredensial tidak disediakan.");
    Serial.println("Kembali ke Mode Access Point (AP).");
    WiFi.mode(WIFI_AP);          // Mengatur ESP8266 sebagai Access Point.
    WiFi.softAP(hostname.c_str()); // Membuat jaringan WiFi dengan `hostname` sebagai SSID.
    IPAddress myIP = WiFi.softAPIP(); // Mendapatkan alamat IP AP.
    Serial.print("Alamat IP AP: ");
    Serial.println(myIP);        // Mencetak alamat IP dari Access Point.
    delay(1000);
  }

  // --- Pengaturan Server Web ---
  server.on("/", HTTP_handleRoot);         // Menangani permintaan ke URL root ("/") dengan fungsi `HTTP_handleRoot`.
  server.on("/distance", handleGetDistance); // Menangani permintaan ke "/distance" untuk mendapatkan jarak sensor.
  server.onNotFound(handleNotFound);       // Menangani URL yang tidak dikenal (HTTP 404 Not Found) dengan fungsi `handleNotFound`.
  server.begin();                          // Memulai server web agar mulai mendengarkan permintaan.
  Serial.println("Server HTTP dimulai");

  // --- Pengaturan OTA (Over-The-Air) Updates ---
  // Memungkinkan mengunggah firmware baru secara nirkabel melalui Arduino IDE.
  ArduinoOTA.setHostname(hostname.c_str()); // Mengatur hostname untuk OTA (sesuai hostname WiFi).
  ArduinoOTA.begin();                       // Memulai layanan OTA.
  Serial.println("OTA diaktifkan.");
}

void loop() {
  ArduinoOTA.handle();      // Memproses permintaan pembaruan OTA yang masuk.
  server.handleClient();    // Memproses semua permintaan HTTP yang masuk dari klien.

  // --- Pembacaan Sensor Jarak ---
  unsigned long currentMillis = millis(); // Mendapatkan waktu saat ini.
  if (currentMillis - lastSensorReadMillis >= sensorReadInterval) { // Memeriksa apakah sudah waktunya membaca sensor lagi.
    lastSensorReadMillis = currentMillis; // Perbarui waktu terakhir pembacaan sensor.

    unsigned int distance_cm = sonar.ping_cm(); // Membaca jarak dalam cm menggunakan fungsi `ping_cm()` dari NewPing.
    // NewPing mengembalikan 0 jika tidak ada echo (objek di luar jangkauan MAX_DISTANCE_CM)
    // atau jika objek terlalu dekat (di bawah min_cm, default 2cm).
    // Jika hasilnya 0, kita set ke MAX_DISTANCE_CM + 1 untuk menandakan "tidak ada objek" atau "sangat jauh" dalam konteks UI.
    if (distance_cm == 0) {
      currentDistanceCm = MAX_DISTANCE_CM + 1; // Mengasumsikan sangat jauh jika 0 atau di luar jangkauan.
    } else {
      currentDistanceCm = distance_cm; // Memperbarui variabel global `currentDistanceCm` dengan jarak yang terbaca.
    }

    // Pemicu Buzzer
    // Buzzer akan berbunyi jika sensor diaktifkan (`sensorEnabled` = true) dan salah satu kondisi terpenuhi:
    // 1. Dalam mode manual/otonom dan objek sangat dekat (<10cm) dan jarak > 0 (bukan nilai '0' yang menandakan luar jangkauan/terlalu dekat).
    // 2. Dalam mode PET dan objek dalam rentang PET (5-15cm) dan jarak > 0.
    if (sensorEnabled) {
      if (!petMode && currentDistanceCm > 0 && currentDistanceCm < 10) { // Mode non-PET, objek sangat dekat
        Serial.print("Objek terdeteksi pada ");
        Serial.print(currentDistanceCm);
        Serial.println(" cm. Membunyikan klakson (non-PET).");
        BeepHorn(); // Memanggil fungsi untuk membunyikan buzzer.
      } else if (petMode && currentDistanceCm > 0 && currentDistanceCm <= PET_MODE_APPROACH_MAX) { // Mode PET, objek dalam rentang deteksi
        Serial.print("Objek terdeteksi pada ");
        Serial.print(currentDistanceCm);
        Serial.println(" cm. Membunyikan klakson (PET Mode).");
        BeepHorn(); // Memanggil fungsi untuk membunyikan buzzer.
      }
    }
  }

  // --- Memproses Perintah (Selalu aktif, termasuk selama mode otonom/PET) ---
  // Memeriksa apakah ada argumen "State" dalam permintaan HTTP.
  if (server.hasArg("State")) {
    command = server.arg("State"); // Mengambil nilai perintah dari argumen "State".
    Serial.print("Perintah diterima: ");
    Serial.println(command); // Mencetak perintah yang diterima ke Serial monitor.

    // Perintah STOP ('S') selalu diproses untuk menghentikan robot dan menonaktifkan mode cerdas.
    if (command == "S") {
      autonomousMode = false; // Menonaktifkan mode otonom.
      petMode = false;        // Menonaktifkan mode PET.
      Stop();                 // Menghentikan motor.
      Serial.println("STOP!");
    }
    // Perintah 'w' untuk Sensor OFF (Lampu OFF).
    else if (command == "w") {
      sensorEnabled = false;  // Menonaktifkan sensor jarak.
      TurnLightOff();         // Mematikan LED indikator.
      Serial.println("Sensor jarak dinonaktifkan dan lampu mati!");
      // Jika sensor dimatikan secara manual, mode cerdas (otonom/PET) juga harus dimatikan.
      if (autonomousMode || petMode) {
        autonomousMode = false;
        petMode = false;
        Stop(); // Berhenti saat mode cerdas dinonaktifkan.
        Serial.println("Mode Otomatis/PET juga dinonaktifkan saat sensor dimatikan.");
      }
    }
    // Perintah untuk mengaktifkan/menonaktifkan mode otonom.
    else if (command == "A") { // 'A' untuk Autonomous Mode ON.
      autonomousMode = true;  // Mengaktifkan mode otonom.
      petMode = false;        // Memastikan mode PET nonaktif.
      sensorEnabled = true;   // Mengaktifkan sensor saat mode otonom ON.
      TurnLightOn();          // Menyalakan LED sebagai indikator mode otonom aktif.
      Serial.println("Mode Otomatis AKTIF!");
      // Robot mulai mundur saat mode otomatis aktif (karena sensor di belakang).
      Backward();
    }
    else if (command == "a") { // 'a' untuk Autonomous Mode OFF.
      autonomousMode = false; // Menonaktifkan mode otonom.
      Stop();                 // Berhenti saat mode otonom dinonaktifkan.
      Serial.println("Mode Otomatis NONAKTIF!");
      // Status `sensorEnabled` tidak diubah oleh perintah 'a', tetap pada status sebelumnya.
    }
    // Perintah untuk mengaktifkan/menonaktifkan mode PET.
    else if (command == "P") { // 'P' untuk Pet Mode ON.
      petMode = true;         // Mengaktifkan mode PET.
      autonomousMode = false; // Memastikan mode otonom nonaktif.
      sensorEnabled = true;   // Mengaktifkan sensor saat mode PET ON.
      TurnLightOn();          // Menyalakan LED sebagai indikator mode PET aktif.
      Serial.println("Mode PET AKTIF!");
      Stop(); // Mulai dengan berhenti dan menunggu objek masuk ke jangkauan.
    }
    else if (command == "p") { // 'p' untuk Pet Mode OFF.
      petMode = false;        // Menonaktifkan mode PET.
      Stop();                 // Berhenti saat mode PET dinonaktifkan.
      Serial.println("Mode PET NONAKTIF!");
      // Status `sensorEnabled` tidak diubah oleh perintah 'p', tetap pada status sebelumnya.
    }
    // Perintah untuk mengaktifkan sensor ON (secara manual).
    else if (command == "W") { // 'W' untuk Sensor ON (Lampu ON).
      sensorEnabled = true;   // Mengaktifkan sensor jarak.
      TurnLightOn();          // Menyalakan LED.
      Serial.println("Sensor jarak diaktifkan dan lampu menyala!");
    }
    // Hanya memproses perintah manual motor lainnya jika robot TIDAK dalam mode otonom/PET.
    // Ini mencegah konflik antara kontrol manual dan logika mode cerdas.
    else if (!autonomousMode && !petMode) {
      if (command == "F") { Forward(); /* Status sensor tidak berubah */ }
      else if (command == "B") { Backward(); /* Status sensor tidak berubah */ }
      else if (command == "R") { TurnRight(); /* Status sensor tidak berubah */ }
      else if (command == "L") { TurnLeft(); /* Status sensor tidak berubah */ }
      else if (command == "V") { BeepHorn(); /* Status sensor tidak berubah */ }
    }
  }

  // --- Logika Mode Otomatis / PET ---
  // Fungsi mode cerdas hanya dipanggil jika mode tersebut aktif.
  if (autonomousMode) {
    AutonomousMode(); // Panggil fungsi untuk mode otonom.
  } else if (petMode) {
    PetMode(); // Panggil fungsi untuk mode PET.
  }
}

// --- HTTP Request Handlers ---

// Fungsi untuk menangani permintaan ke URL root "/" (halaman web utama).
void HTTP_handleRoot(void) {
  // Menggunakan R"rawliteral(...)rawliteral" untuk string HTML multi-baris
  // Ini adalah cara yang nyaman untuk mendefinisikan string panjang tanpa masalah escape karakter.
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
    overflow: hidden; /* Mencegah scrolling halaman */
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; /* Font modern */
    background-color: #e9ecef; /* Warna latar belakang lebih terang */
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center; /* Memusatkan konten secara vertikal */
    text-align: center;
  }
  .header {
    width: 90%;
    max-width: 800px; /* Lebar maksimum yang ditingkatkan untuk desktop */
    background-color: #0056b3; /* Header biru tua */
    color: white;
    padding: 20px 0;
    border-radius: 12px; /* Sudut lebih membulat */
    margin-bottom: 25px;
    flex-shrink: 0; /* Mencegah header menyusut */
    box-shadow: 0 6px 15px rgba(0,0,0,0.2); /* Bayangan lebih kuat */
  }
  .header h1 {
    margin: 0;
    font-size: 2em; /* Ukuran font header lebih besar */
    letter-spacing: 1px;
  }
  .header p {
    margin: 8px 0 0;
    font-size: 1.1em;
    opacity: 0.9;
  }
  .distance-display {
    font-size: 2.2em; /* Ukuran font jarak lebih besar */
    font-weight: bold;
    color: #212529; /* Teks lebih gelap */
    margin-bottom: 25px;
    padding: 18px 35px;
    background-color: #ffffff; /* Latar belakang putih */
    border-radius: 12px;
    box-shadow: 0 4px 10px rgba(0,0,0,0.1);
    flex-shrink: 0;
    min-width: 200px; /* Memastikan tidak terlalu kecil */
  }
  .container {
    display: grid;
    grid-template-columns: repeat(3, 1fr); /* Tiga kolom dengan lebar yang sama */
    gap: 20px; /* Jarak antar tombol lebih besar */
    width: 95%;
    max-width: 800px; /* Lebar maksimum yang ditingkatkan untuk desktop */
    height: auto; /* Membiarkan konten menentukan tinggi untuk fleksibilitas */
    padding: 30px; /* Padding lebih besar */
    background-color: #f8f9fa; /* Latar belakang abu-abu terang */
    border-radius: 12px;
    box-shadow: 0 8px 20px rgba(0,0,0,0.18); /* Bayangan lebih kuat */
    align-items: stretch; /* Memastikan item mengisi tinggi sel grid */
    justify-items: stretch; /* Memastikan item mengisi lebar sel grid */
  }
  .button {
    width: 100%;
    height: auto; /* Menyesuaikan tinggi secara dinamis */
    min-height: 80px; /* Tinggi minimum untuk target sentuh yang lebih baik */
    padding: 15px 0;
    font-size: 2.5em; /* Ukuran font tombol lebih besar */
    font-weight: bold;
    color: white;
    background-color: #28a745; /* Warna hijau Bootstrap */
    border: none;
    border-radius: 10px;
    cursor: pointer;
    text-align: center;
    text-decoration: none;
    display: flex;
    justify-content: center;
    align-items: center;
    transition: background-color 0.2s ease, transform 0.1s ease; /* Transisi halus */
    -webkit-tap-highlight-color: rgba(0,0,0,0); /* Menghilangkan highlight saat disentuh di mobile */
  }
  .button:active {
    background-color: #218838; /* Lebih gelap saat aktif */
    transform: translateY(2px); /* Efek penekanan sedikit */
  }

  /* Warna tombol spesifik */
  .button.stop { background-color: #dc3545; } /* Merah Bootstrap */
  .button.stop:active { background-color: #c82333; }
  .button.horn { background-color: #ffc107; color: #343a40; } /* Kuning Bootstrap, teks gelap */
  .button.horn:active { background-color: #e0a800; }
  .button.sensor { background-color: #6f42c1; } /* Ungu Bootstrap */
  .button.sensor:active { background-color: #5a2e9e; }
  .button.autonomous { background-color: #20c997; } /* Hijau kebiruan Bootstrap */
  .button.autonomous:active { background-color: #19a47c; }
  .button.pet { background-color: #fd7e14; } /* Oranye Bootstrap */
  .button.pet:active { background-color: #e46001; }

  /* Tombol arah */
  .directional-button {
    background-color: #007bff; /* Biru Bootstrap */
  }
  .directional-button:active {
    background-color: #0056b3;
  }
  .button.icon-button {
    font-size: 1.4em; /* Ukuran font disesuaikan untuk teks multi-baris */
    padding: 12px;
    line-height: 1.3;
    min-height: 60px; /* Memastikan tombol yang lebih kecil tetap dapat disentuh */
  }
  .button.stop-park {
    grid-column: span 3; /* Membentang tiga kolom */
    font-size: 3.2em; /* Ukuran font tombol STOP lebih besar */
    background-color: #6c757d; /* Abu-abu Bootstrap */
  }
  .button.stop-park:active {
    background-color: #5a6268;
  }
  .footer-text {
    margin-top: 25px;
    font-size: 1em;
    color: #6c757d; /* Abu-abu redup untuk footer */
    flex-shrink: 0;
  }

  /* Media Queries untuk Responsivitas */
  /* Untuk mobile potret */
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
      height: auto; /* Memungkinkan tinggi dinamis berdasarkan konten */
      max-height: none; /* Menghilangkan batasan max-height */
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

  /* Untuk mobile/tablet lanskap */
  @media (max-height: 600px) and (orientation: landscape) {
    body {
      padding: 10px;
      justify-content: flex-start; /* Sejajarkan konten ke atas dalam mode lanskap untuk menghemat ruang */
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
      height: auto; /* Memungkinkan tinggi dinamis */
      max-height: none; /* Menghilangkan batasan tinggi tetap */
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

  /* Optimasi lebih lanjut untuk layar sangat lebar (misal: monitor besar) */
  @media (min-width: 1024px) {
    .header, .distance-display, .container {
      padding-left: 40px;
      padding-right: 40px;
      max-width: 900px; /* Lebih lebar untuk layar sangat besar */
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
  // Fungsi untuk mengirim perintah kontrol ke robot melalui permintaan HTTP GET.
  function sendCommand(cmd) {
    var xhttp = new XMLHttpRequest(); // Membuat objek XMLHttpRequest baru.
    xhttp.onreadystatechange = function() {
      // Fungsi ini akan dipanggil setiap kali status request berubah.
      if (this.readyState == 4 && this.status == 200) {
        // Jika request selesai (readyState 4) dan berhasil (status 200 OK),
        // bisa log respons ke konsol (saat ini dikomentari).
        // console.log("Command '" + cmd + "' sent. Response: " + this.responseText);
      }
    };
    // Menggunakan alamat IP robot yang sama dengan halaman ini.
    xhttp.open("GET", "/?State=" + cmd, true); // Membuka permintaan GET ke URL root dengan argumen "State".
    xhttp.send(); // Mengirim permintaan.
  }

  // Fungsi untuk mendapatkan jarak sensor dari robot.
  function getDistance() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        let distance = this.responseText; // Mendapatkan teks respons (jarak).
        // Jika jarak yang dikembalikan adalah MAX_DISTANCE_CM + 1, itu berarti objek di luar jangkauan atau terlalu dekat (0 dari NewPing).
        if (distance == %MAX_DISTANCE_CM% + 1) {
          document.getElementById("distanceValue").innerText = " > " + %MAX_DISTANCE_CM% + " (Jauh)";
        } else {
          document.getElementById("distanceValue").innerText = distance; // Menampilkan jarak yang terbaca.
        }
      }
    };
    xhttp.open("GET", "/distance", true); // Membuka permintaan GET ke "/distance".
    xhttp.send(); // Mengirim permintaan.
  }

  // Memanggil fungsi getDistance setiap 500ms untuk pembaruan real-time pada tampilan jarak.
  setInterval(getDistance, 500);
</script>
</head>
<body>

  <div class="header">
    <h1>ESP8266 WiFi Robot Car</h1>
    <p>IP: %IP_ADDRESS%</p> <!-- Placeholder untuk alamat IP, akan diganti oleh ESP8266 -->
  </div>

  <div class="distance-display">
    Jarak: <span id="distanceValue">N/A</span> cm <!-- Tampilan jarak sensor -->
  </div>

  <div class="container">
    <div></div> <!-- Sel kosong untuk tata letak grid -->
    <!-- Tombol Maju. ontouchstart untuk mulai bergerak, onmouseup/ontouchend untuk berhenti. -->
    <button class="button directional-button" ontouchstart="sendCommand('F')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8593;</button>
    <div></div>

    <!-- Tombol Kiri. -->
    <button class="button directional-button" ontouchstart="sendCommand('L')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8592;</button>
    <div style="display: flex; flex-direction: column; gap: 10px;">
      <!-- Klakson -->
      <button class="button icon-button horn" ontouchstart="sendCommand('V')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#128266; Klakson</button>
      <!-- Lampu ON (juga mengaktifkan sensor) -->
      <button class="button icon-button sensor" onclick="sendCommand('W')">Lampu ON</button>
      <!-- Lampu OFF (juga menonaktifkan sensor) -->
      <button class="button icon-button sensor" onclick="sendCommand('w')">Lampu OFF</button>
      <!-- Mode Otomatis ON -->
      <button class="button icon-button autonomous" onclick="sendCommand('A')">MODE OTOMATIS ON</button>
      <!-- Mode Otomatis OFF -->
      <button class="button icon-button autonomous" onclick="sendCommand('a')">MODE OTOMATIS OFF</button>
      <!-- Mode Peliharaan ON -->
      <button class="button icon-button pet" onclick="sendCommand('P')">MODE PELIHARAAN ON</button>
      <!-- Mode Peliharaan OFF -->
      <button class="button icon-button pet" onclick="sendCommand('p')">MODE PELIHARAAN OFF</button>
    </div>
    <!-- Tombol Kanan -->
    <button class="button directional-button" ontouchstart="sendCommand('R')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8594;</button>

    <div></div>
    <!-- Tombol Mundur -->
    <button class="button directional-button" ontouchstart="sendCommand('B')" onmouseup="sendCommand('S')" ontouchend="sendCommand('S')">&#8595;</button>
    <div></div>
    
    <!-- Tombol STOP besar -->
    <button class="button stop-park" onclick="sendCommand('S')">STOP</button>
  </div>

  <p class="footer-text">Kontrol robot Anda melalui WiFi. Gunakan tombol panah untuk arah, dan tombol di tengah untuk fungsi tambahan.</p>

</body>
</html>
)rawliteral";

  // Mendapatkan alamat IP saat ini untuk ditampilkan di halaman.
  String ipAddress;
  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString(); // Jika terhubung sebagai STA.
  } else {
    ipAddress = WiFi.softAPIP().toString(); // Jika dalam mode AP.
  }
  // Mengganti placeholder di HTML dengan alamat IP aktual.
  html.replace("%IP_ADDRESS%", ipAddress);
  // Mengganti placeholder MAX_DISTANCE_CM untuk tampilan jarak jauh.
  html.replace("%MAX_DISTANCE_CM%", String(MAX_DISTANCE_CM));

  // Mengirim halaman HTML ke klien dengan status HTTP 200 OK dan tipe konten text/html.
  server.send(200, "text/html", html);
}

// Fungsi untuk menangani permintaan URI yang tidak dikenal (HTTP 404 Not Found).
void handleNotFound() {
  server.send(404, "text/plain", "404: Tidak Ditemukan"); // Mengirim respons 404.
}

// --- Fungsi untuk menangani permintaan mendapatkan jarak sensor ---
void handleGetDistance() {
  // Mengirim nilai `currentDistanceCm` sebagai teks biasa dengan status HTTP 200 OK.
  server.send(200, "text/plain", String(currentDistanceCm));
}

// --- Motor Control Functions ---

// Menggerakkan kedua motor maju.
void Forward() {
  // Motor Kiri (Motor A) - Maju
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B) - Maju
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Menggerakkan kedua motor mundur.
void Backward() {
  // Motor Kiri (Motor A) - Mundur
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Motor Kanan (Motor B) - Mundur
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Membuat robot berbelok ke kanan di tempat (Motor Kiri Maju, Motor Kanan Mundur).
void TurnRight() {
  // Motor Kiri (Motor A) - Maju
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B) - Mundur
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
}

// Membuat robot berbelok ke kiri di tempat (Motor Kiri Mundur, Motor Kanan Maju).
void TurnLeft() {
  // Motor Kiri (Motor A) - Mundur
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);

  // Motor Kanan (Motor B) - Maju
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
}

// Menghentikan kedua motor segera.
void Stop() {
  // Motor Kiri (Motor A)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);

  // Motor Kanan (Motor B)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
}

// Mengaktifkan buzzer (klakson) untuk durasi singkat.
void BeepHorn() {
  digitalWrite(BUZZER_PIN, HIGH); // Menyalakan buzzer.
  delay(150);                     // Bunyi selama 150ms (ini akan sedikit memblokir eksekusi kode lain).
  digitalWrite(BUZZER_PIN, LOW);  // Mematikan buzzer.
  delay(80);                      // Jeda singkat 80ms (ini juga memblokir).
}

// Fungsi untuk menyalakan LED tambahan (terhubung ke D2).
void TurnLightOn() {
  digitalWrite(LED_PIN, HIGH);
}

// Fungsi untuk mematikan LED tambahan (terhubung ke D2).
void TurnLightOff() {
  digitalWrite(LED_PIN, LOW);
}

// Fungsi untuk Mode Otonom (disesuaikan untuk sensor di belakang robot).
void AutonomousMode() {
  // Memastikan sensor diaktifkan di mode ini.
  sensorEnabled = true;
  TurnLightOn(); // Menyalakan LED sebagai indikator.

  // Logika penghindaran halangan dasar: robot bergerak mundur, sensor di belakang.
  // Jika objek sangat dekat di belakang.
  if (currentDistanceCm <= OBSTACLE_STOP_DISTANCE && currentDistanceCm != 0) {
    Serial.println("Mode Otomatis: Objek sangat dekat di belakang! Berhenti & Maju...");
    Stop();       // Hentikan robot.
    delay(200);   // Berhenti sebentar.
    Forward();    // **Maju** untuk menjauh dari halangan di belakang.
    delay(500);   // Maju sebentar.
    Stop();       // Hentikan lagi.
    delay(100);
    // Kemudian putar secara acak untuk mencari jalur lain.
    if (random(2) == 0) { // Pilih acak apakah belok kiri atau kanan.
      TurnLeft();
      Serial.println("Mode Otomatis: Belok Kiri.");
    } else {
      TurnRight();
      Serial.println("Mode Otomatis: Belok Kanan.");
    }
    delay(500); // Belok sebentar.
    Stop();     // Berhenti setelah belok.
    delay(100);
  }
  // Jika objek agak dekat di belakang, coba belok ringan.
  else if (currentDistanceCm > OBSTACLE_STOP_DISTANCE && currentDistanceCm <= OBSTACLE_TURN_DISTANCE && currentDistanceCm != 0) {
    Serial.println("Mode Otomatis: Objek agak dekat di belakang! Sesuaikan arah...");
    // Coba belok ringan ke salah satu sisi secara acak.
    if (random(2) == 0) {
      TurnLeft();
    } else {
      TurnRight();
    }
    delay(300); // Belok sebentar.
    Stop();
    delay(50);
  }
  // Jika tidak ada objek di belakang dalam jarak aman, terus mundur.
  else {
    Serial.println("Mode Otomatis: Bergerak Mundur.");
    Backward(); // Robot terus mundur (gerakan dasar mode otomatis).
  }
}

// Fungsi untuk Mode PET (Peliharaan) - Disesuaikan untuk mendekati objek dengan sensor di belakang.
void PetMode() {
  sensorEnabled = true; // Memastikan sensor aktif di mode ini.
  TurnLightOn();        // Menyalakan LED sebagai indikator.

  // Jika objek terdeteksi dalam rentang mendekat (misal 5-15cm).
  if (currentDistanceCm > PET_MODE_STOP_DISTANCE && currentDistanceCm <= PET_MODE_APPROACH_MAX && currentDistanceCm != 0) {
    Serial.print("Mode PET: Objek pada ");
    Serial.print(currentDistanceCm);
    Serial.println(" cm. Mendekat (Mundur)...");
    Backward(); // **Bergerak mundur** untuk mendekati objek (karena sensor di belakang).
    // Buzzer otomatis akan berbunyi karena kondisi di `loop()` sekarang mencakup mode PET.
  }
  // Jika objek terlalu dekat (misal <= 5cm), berhenti total.
  else if (currentDistanceCm <= PET_MODE_STOP_DISTANCE && currentDistanceCm != 0) {
    Serial.print("Mode PET: Objek terlalu dekat pada ");
    Serial.print(currentDistanceCm);
    Serial.println(" cm! BERHENTI.");
    Stop(); // Hentikan robot.
    // Buzzer otomatis akan berbunyi karena kondisi di `loop()` sekarang mencakup mode PET.
  }
  // Jika objek di luar jangkauan deteksi (lebih dari PET_MODE_APPROACH_MAX atau tidak terdeteksi).
  else {
    Serial.println("Mode PET: Tidak ada objek dalam jangkauan. Diam.");
    Stop(); // Berhenti dan tunggu objek masuk ke jangkauan.
  }
}
