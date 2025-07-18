
#### **Adım 1: Gereksinimleri Anlama**
Bu sistem, Suibo kartı ile çalışan bir su altı aracı (örneğin, Suibo Derin Commander) için tasarlanmıştır. Gerekli bileşenler:
- **Donanım**:
  - Suibo kartı (yerleşik ivmeölçer ve jiroskop ile).
  - 8 pervaneli (Crab 8 konfigürasyonu) su altı aracı.
  - USB seri port kablosu.
  - Oyun kolu (örneğin, Xbox veya PS4 kontrol cihazı).
- **Yazılım**:
  - Python 3.8+ (PyQt5, pygame, pyserial kütüphaneleri).
  - Arduino IDE.
  - Adafruit LSM6DS ve Adafruit_Sensor kütüphaneleri (Suibo’nun IMU’su için).
- **Bağlantı**: Seri port (örneğin, COM1) üzerinden iletişim, 115200 baud rate.

#### **Adım 2: Yazılım Kurulumları**
1. **Python Ortamını Hazırlama**:
   - Python 3.8 veya üstü bir sürümü yükleyin (https://www.python.org/).
   - Gerekli kütüphaneleri yüklemek için terminalde şu komutları çalıştırın:
     ```bash
     pip install pyqt5 pygame pyserial
     ```
   - Kod dosyalarını (`main.py`, `Controller_Input_GUI.py`, `PID_Editor_GUI.py`, `otonom_starter.py`, `serial_manager.py`) bir klasöre yerleştirin.

2. **Arduino Ortamını Hazırlama**:
   - Arduino IDE’yi yükleyin (https://www.arduino.cc/en/software).
   - Güncellenmiş `suibo_controller.ino` dosyasını Arduino IDE ile açın.
   - Gerekli kütüphaneleri yükleyin:
     - `Servo.h` ve `Wire.h` (Arduino IDE ile gelir).
     - `Adafruit_Sensor` ve `Adafruit_LSM6DS` kütüphaneleri:
       ```bash
       Arduino IDE -> Sketch -> Include Library -> Manage Libraries -> "Adafruit LSM6DS" ve "Adafruit_Sensor" arat ve kur
       ```
   - Suibo kartını bilgisayara bağlayın ve doğru COM portunu seçin (örneğin, COM1).

#### **Adım 3: Donanım Kurulumu**
1. **Suibo Kartı ve Motor Bağlantıları**:
   - 8 adet ESC’yi (Electronic Speed Controller) Suibo kartının PWM pinlerine bağlayın (pinler: 3, 5, 6, 9, 10, 11, 12, 13).
   - ESC’lerin motorlara ve güç kaynağına doğru bağlandığından emin olun.
   - Motorların nötr pozisyonda (1500 µs PWM) olduğundan emin olun.

2. **IMU Bağlantısı**:
   - Suibo kartının yerleşik ivmeölçer ve jiroskopu kullanıldığından, ek sensör bağlantısı gerekmez.
   - Kartın I2C pinlerinin (SCL, SDA) doğru şekilde yapılandırıldığından emin olun (varsayılan olarak bağlı olmalıdır).

3. **Seri Port Bağlantısı**:
   - Suibo kartını USB kablosuyla bilgisayara bağlayın.
   - Doğru COM portunu not edin (örneğin, COM1).

#### **Adım 4: Arduino Kodunu Yükleme**
1. Arduino IDE’de `suibo_controller.ino` dosyasını açın.
2. Kodda şu ayarları kontrol edin:
   - Seri port baud rate: `Serial.begin(115200)` (GUI ile eşleşmeli).
   - Motor pinleri: `motorPins` dizisi (PWM pinlerinizi doğrulayın).
   - IMU ayarları: `initIMU()` fonksiyonu, Suibo’nun yerleşik IMU’su için yapılandırılmıştır (416 Hz, 8g, 1000 dps).
3. Kodu Suibo kartına yükleyin:
   - Arduino IDE’de "Upload" butonuna tıklayın.
   - Yükleme başarılıysa, seri monitörde (115200 baud) "IMU initialized" mesajını görmelisiniz.

#### **Adım 5: Python Uygulamasını Çalıştırma**
1. **Seri Port Ayarını Güncelleme**:
   - `main.py` içinde `serial_manager = SerialPortManager(port='COM1', baudrate=115200)` satırındaki `port` değerini Suibo kartınızın bağlı olduğu COM portuyla değiştirin (örneğin, `COM3`).

2. **Uygulamayı Başlatma**:
   - Terminalde proje klasörüne gidin ve şu komutu çalıştırın:
     ```bash
     python main.py
     ```
   - Üç pencere açılacaktır:
     - **PID Kontrol Arayüzü**: PID parametrelerini ayarlamak için.
     - **Gamepad Kontrol Arayüzü**: Oyun kolu ile kontrol için.
     - **Otonom Mod Kontrolü**: Otonom modu başlatmak/durdurmak için.

#### **Adım 6: Sistemi Kullanma**
1. **Gamepad Kontrolü**:
   - Oyun kolunu bilgisayara bağlayın (USB veya Bluetooth).
   - `Controller_Input_GUI` penceresinde:
     - Bağlantı durumu "Gamepad: Bağlı" olarak güncellenir (yeşil renk).
     - Joystick eksenleri progress bar’larda, düğmeler ise renk değişimiyle (yeşil=aktif, gri=pasif) gösterilir.
     - "Kalibrasyon Yap" butonuna tıklayarak oyun kolu kalibrasyonunu test edebilirsiniz.
   - Joystick hareketleri ve düğmeler, seri port üzerinden Suibo kartına gönderilir ve motorlar buna göre hareket eder.

2. **PID Ayarları**:
   - `PID_Editor_GUI` penceresinde:
     - Yaw, pitch ve roll için Kp, Ki, Kd değerlerini girin (örneğin, Kp=1.0, Ki=0.1, Kd=0.5).
     - "Tüm PID Değerlerini Gönder" butonuna tıklayın.
     - Değerler seri port üzerinden Suibo kartına gönderilir ve PID kontrolü için kullanılır.
   - Bağlantı durumu yeşil LED ile gösterilir.

3. **Otonom Mod**:
   - `otonom_starter` penceresinde:
     - "Otonom Mod Başlat" butonuna tıklayın; bu, Suibo kartına `AUTONOM:START` komutunu gönderir ve PID kontrollü otonom stabilizasyon başlar.
     - "Rota Yükle" butonu, önceden tanımlı bir rotayı yükler (şu an yalnızca varsayılan rota destekleniyor).
     - Durumu durdurmak için "Otonom Modu Durdur" butonuna tıklayın; motorlar nötr konuma döner.

4. **Seri Port İletişimi**:
   - `serial_manager.py`, tüm veri alışverişini thread-safe şekilde yönetir.
   - Suibo kartından gelen IMU verileri (örneğin, `IMU:10.50,5.25,3.15`), `otonom_starter` arayüzünde durum etiketi olarak görünür.

#### **Adım 7: Test ve Hata Ayıklama**
1. **Oyun Kolu Testi**:
   - Oyun kolunu hareket ettirin ve `Controller_Input_GUI`’deki progress bar’ların güncellendiğini doğrulayın.
   - Düğmelere basın; etiketlerin yeşile dönmesi gerekir.

2. **PID Kontrol Testi**:
   - Arduino seri monitörünü açarak (115200 baud) gönderilen PID komutlarını kontrol edin (örneğin, `PID:yaw,1.000,0.100,0.500;...`).
   - IMU verilerinin seri monitörde düzenli olarak göründüğünü doğrulayın (örneğin, `IMU:10.50,5.25,3.15`).

3. **Otonom Mod Testi**:
   - Otonom modu başlatarak motorların PID kontrolüyle hareket ettiğini gözlemleyin.
   - IMU verilerinin stabilizasyon için kullanıldığını seri monitördeki geri bildirimle doğrulayın.

4. **Hata Ayıklama**:
   - Seri port bağlantı sorunları için `serial_manager.py`’deki hata mesajlarını kontrol edin.
   - Oyun kolu bağlantısı başarısız olursa, pygame’in doğru şekilde yüklendiğinden ve oyun kolunun bağlı olduğundan emin olun.
   - IMU verileri gelmiyorsa, Suibo kartının yerleşik IMU’sunun doğru yapılandırıldığını ve `initIMU()` fonksiyonunun çalıştığını kontrol edin.

#### **Adım 8: Özelleştirme ve Geliştirme**
1. **IMU Kalibrasyonu**:
   - Suibo’nun yerleşik IMU’su için kalibrasyon gerekiyorsa, `initIMU()` fonksiyonuna kalibrasyon rutinleri ekleyin (örneğin, jiroskop sıfırlama veya ivmeölçer ofset ayarı).

2. **Rota Yükleme**:
   - `otonom_starter.py`’deki `load_route()` fonksiyonunu, gerçek rota verileriyle (örneğin, bir JSON dosyası veya Derin Commander API’si) entegre edin.

3. **Motor Haritalama**:
   - `applyGamepadControl()` fonksiyonunda motor haritalamasını, aracınızın fiziksel konfigürasyonuna göre özelleştirin.

4. **GUI Özelleştirme**:
   - `Controller_Input_GUI.py`’de daha fazla eksen veya düğme eklemek için `create_axis_group()` ve `create_button_group()` fonksiyonlarını güncelleyin.

#### **Adım 9: Yaygın Sorunlar ve Çözümler**
- **Oyun Kolu Algılanmıyor**: pygame’in doğru yüklendiğini ve oyun kolunun bağlı olduğunu kontrol edin. `pygame.joystick.get_count()`’u test edin.
- **Seri Port Hatası**: Doğru COM portunu kullandığınızdan emin olun. `serial_manager.py`’deki `reconnect()` fonksiyonu otomatik yeniden bağlanmayı dener.
- **IMU Verileri Yanlış**: Suibo kartının IMU’sunun düzgün çalıştığını ve kütüphanelerin doğru yüklendiğini kontrol edin. `lsm6ds.begin()`’in başarılı olduğunu seri monitörde doğrulayın.
- **Motorlar Çalışmıyor**: ESC’lerin kalibre edildiğinden ve PWM sinyallerinin doğru aralıkta (1270-1730 µs) olduğundan emin olun.

#### **Adım 10: Son Notlar**
- Güncellenmiş kod, Suibo kartının yerleşik IMU’sunu kullanarak daha entegre bir çözüm sunar.
- Gerçek bir su altı aracıyla test etmeden önce, tüm motor ve sensör bağlantılarını kuru bir ortamda test edin.
- Arduino seri monitörünü ve Python konsol çıktılarını aktif olarak izleyerek hata ayıklamayı kolaylaştırın.
