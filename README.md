### Projenin Özeti
**Proje Genel Bakış**: Bu proje, Degz Robotics’in **Suibo kontrol kartı** ile çalışan, **Crab 8** itici konfigürasyonunda bir insansız sualtı aracı (UUV) için kontrol sistemidir. Sistem, bir oyun kolu ile manuel kontrol, PID kontrolü ile otonom stabilizasyon ve gerçek zamanlı sensör geri bildirimi sağlar. Python tabanlı bir GUI uygulaması üzerinden yönetilir ve seri iletişim (UART, 115200 baud) ile komut gönderip IMU verileri alır. Navigasyon, inceleme veya keşif gibi görevleri destekler.

#### Bileşenler
1. **Suibo Kontrol Kartı**:
   - **Donanım**: ARM M0 mikrodenetleyicisi, 8 motor desteği, 9 eksenli IMU (jiroskop/ivmeölçer, LSM6DS3 varsayımı, manyetometre yok), UART, 3.3–6V güç.
   - **Yazılım**: `suibo_controller.ino` çalıştırır, GUI’den gelen komutları (`PID:`, `!GP:`, `AUTONOM:`) ayrıştırır, iticileri kontrol eder ve IMU geri bildirimi (`IMU:yaw,pitch,roll\n`) gönderir.
2. **Crab 8 İtici Konfigürasyonu**:
   - 8 itici (örn. Blue Robotics T100), ESC’ler aracılığıyla kontrol edilir (PWM 1270–1730 µs):
     - Motorlar 0–3: İleri/geri (X ekseni).
     - Motorlar 4–5: Yanal hareket (Y ekseni).
     - Motorlar 6–7: Derinlik kontrolü (Z ekseni).
   - 4 serbestlik derecesi (DOF): ileri/geri, yanal, derinlik, yunuslama (yaw); PID ile pitch/roll stabilizasyonu.
3. **GUI Uygulaması**:
   - **main.py**: GUI pencerelerini başlatır.
   - **PID_Editor_GUI.py**: Yaw, pitch ve roll için PID parametrelerini ayarlar (`PID:yaw,kp,ki,kd;pitch,kp,ki,kd;roll,kp,ki,kd;\n`).
   - **Controller_Input_GUI.py**: Oyun kolu girişlerini gönderir (`!GP:AXES:a1,a2,a3,a4;BUTTONS:b1,b2,...;\n`).
   - **otonom_starter.py**: Otonom modu açar/kapatır (`AUTONOM:START\n`, `AUTONOM:STOP\n`, `AUTONOM:LOAD_ROUTE:DEFAULT\n`) ve IMU verilerini görüntüler.
   - **serial_manager.py**: UART iletişimini yönetir (115200 baud).
4. **IMU**:
   - LSM6DS3 (varsayım), yunuslama (jiroskop entegrasyonu), pitch ve roll (ivmeölçer) verileri sağlar.
5. **İletişim**:
   - UART (USB veya kablo ile), komut gönderir ve `IMU:yaw,pitch,roll\n` alır.

#### Sistemin İşleyişi
- **Manuel Kontrol**: Oyun kolu girişleri (X/Y/Z eksenleri, düğme 0 durdurma) iticileri doğrudan kontrol eder.
- **Otonom Mod**: IMU verileriyle PID, yunuslama, pitch ve roll’u stabilize eder; rota navigasyonu için yer tutucu.
- **Geri Bildirim**: Suibo, IMU verilerini GUI’ye gönderir (örn. `IMU:123.45,10.20,5.30`).
- **Komutlar**:
  - PID: Stabilizasyon parametrelerini günceller.
  - Oyun Kolu: Eksenleri iticilere, düğmeleri kontrol işlevlerine eşler.
  - Otonom: PID kontrolünü başlatır/durdurur, rota yükler (yer tutucu).

#### Tam İşlevsellik
- **Manuel**: Oyun kolu ile hassas hareket ve yunuslama kontrolü.
- **Otonom**: Türbülanslı sularda stabilizasyon.
- **Geri Bildirim**: Gerçek zamanlı yunuslama, pitch, roll gösterimi.
- **Esneklik**: Derin Commander ile rota navigasyonu eklenebilir.

### Adım Adım Kurulum Rehberi (Tutorial)

#### 1. Donanım Kurulumu
1. **Suibo Kontrol Kartı**:
   - Suibo’yu 3.3–6V ile besleyin (örn. batarya veya USB).
   - USB-to-serial ile bilgisayara bağlayın (örn. COM1).
2. **İticiler (Crab 8)**:
   - 8 ESC’yi PWM pinlerine bağlayın (3, 5, 6, 9, 10, 11, 12, 13; Suibo’nun pin şemasını doğrulayın).
   - Bağlantılar:
     - Motorlar 0–3: İleri/geri.
     - Motorlar 4–5: Yanal hareket.
     - Motorlar 6–7: Derinlik.
   - ESC’leri bataryayla (örn. 12V) besleyin, Suibo ile ortak toprak kullanın.
3. **IMU**:
   - LSM6DS3’ü I2C üzerinden bağlayın (SDA/SCL pinleri, adres 0x6A).
4. **Kablo (Opsiyonel)**:
   - Sualtı için UART destekli kablo kullanın.

#### 2. Yazılım Kurulumu
1. **Arduino IDE**:
   - [arduino.cc](https://www.arduino.cc/en/software) adresinden Arduino IDE 2.x’i indirin ve kurun.
   - ARM M0 desteği için Boards Manager’dan “Arduino SAMD Boards”u ekleyin.
2. **Kütüphaneler**:
   - `Servo` kütüphanesi (Arduino IDE ile gelir).
   - `Adafruit_LSM6DS` kütüphanesini Library Manager’dan kurun.
   - Suibo’nun Derin Commander kütüphanesini (varsa) [Degz Robotics GitHub](https://github.com/degzrobotics) üzerinden yükleyin.
3. **Arduino Kodu Yükleme**:
   - `suibo_controller.ino` dosyasını Arduino IDE’de açın.
   - Kart (örn. Arduino Zero) ve portu (COM1) seçin.
   - Kodu yükleyin, Serial Monitor’da (115200 baud) doğrulamayı kontrol edin.
4. **GUI Bağımlılıkları**:
   - Python 3.x kurun.
   - Gerekli kütüphaneleri yükleyin:
     ```bash
     pip install pyserial tkinter pygame
     ```
   - Tüm GUI dosyalarını (`main.py`, `PID_Editor_GUI.py`, vb.) aynı klasöre yerleştirin.
5. **Dive Control (Opsiyonel)**:
   - [Degz Robotics GitHub](https://github.com/degzrobotics)’dan Dive Control’ü indirin ve kurun.

#### 3. Yapılandırma ve Test
1. **ESC Kalibrasyonu**:
   - İticileri ayırarak ESC’leri açın.
   - `!GP:AXES:128,128,128,128;BUTTONS:0,0,0,0,0,0,0,0;\n` komutuyla nötr PWM (1500 µs) gönderin.
   - ESC kalibrasyon prosedürünü izleyin (örn. maks-min-nötr).
2. **IMU Kalibrasyonu**:
   - `Adafruit_LSM6DS` örnek kalibrasyon kodunu veya Dive Control’ü kullanın.
3. **İletişim Testi**:
   - `main.py`’yi çalıştırın, COM1 bağlantısını doğrulayın.
   - Serial Monitor ile komutları test edin (örn. `PID:yaw,1.0,0.1,0.5;...`, `AUTONOM:START\n`).
4. **Manuel Kontrol Testi**:
   - `Controller_Input_GUI.py` ile oyun kolu hareketlerini test edin (X/Y/Z eksenleri).
5. **Otonom Mod Testi**:
   - `otonom_starter.py` ile otonom modu başlatın/durdurun, IMU geri bildirimini izleyin.
6. **Rota Testi**:
   - Derin Commander ile `AUTONOM:LOAD_ROUTE:DEFAULT`’u uygulayın (varsa).


### Notlar
- **IMU**: `updateIMU`’daki simüle verileri `Adafruit_LSM6DS` ile değiştirin.
- **Derin Commander**: Kütüphane varsa, rota navigasyonunu entegre edin.
- **İticiler**: ESC’lerin PWM aralığını (1270–1730 µs) doğrulayın.
