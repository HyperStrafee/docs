## Arduino Nano + MPU6050 ile Kendini Dengeleyen Robot (NEMA17 Step Motorlar)

Bu proje, Arduino Nano, MPU6050 IMU ve iki adet NEMA17 step motor (A4988/DRV8825 sürücü) kullanarak iki teker üzerinde kendini dengeleyen bir robot örneği sunar. Kod, ek kütüphane gerektirmez (yalnızca Wire).

### Donanım
- **Mikrodenetleyici**: Arduino Nano (ATmega328P)
- **IMU**: MPU6050 (I2C)
- **Motorlar**: 2x NEMA17 Step Motor
- **Sürücüler**: 2x A4988 veya DRV8825 (ayrı ayrı sol ve sağ motor için)
- **Güç**: Motorlar için uygun LiPo/NiMH (sürücüler için yeterli akım ve soğutma), Arduino için 5V

### Bağlantılar
- **I2C (MPU6050)**: VCC->5V, GND->GND, SDA->A4, SCL->A5, AD0->GND (adres 0x68)
- **A4988/DRV8825 Sol Motor**:
  - STEP -> D3
  - DIR  -> D4
  - EN   -> D7 (ortak enable)
  - MS1/MS2/MS3: boş bırakabilir veya LOW/LOW/LOW (tam adım). Kodda pinler 255 ise kontrol edilmiyor.
  - VMOT/VDD/GND: sürücü şemasına göre (diyot ve kondansatörleri unutmayın)
- **A4988/DRV8825 Sağ Motor**:
  - STEP -> D5
  - DIR  -> D6
  - EN   -> D7 (ortak enable)

Not: Yönler teker kablolamasına göre değişebilir. Kodda `LEFT_DIR_INVERTED` ve `RIGHT_DIR_INVERTED` ile düzeltin.

### Yazılım Dosyaları
- `SelfBalanceRobot.ino`: Tüm kontrol döngüsü, MPU6050 okuma, tamamlayıcı filtre, PID ve step pulse üretimi.

### Derleme ve Yükleme
1. Arduino IDE veya arduino-cli ile kartı `Arduino Nano` ve işlemciyi doğru seçin (ATmega328P / Old Bootloader olabilir).
2. `SelfBalanceRobot.ino` dosyasını açın ve parametreleri kendi donanımınıza göre güncelleyin.
3. Derleyip karta yükleyin.

### Konfigürasyon (koddaki üst bölüm)
- **PINLER**: `PIN_LEFT_STEP`, `PIN_LEFT_DIR`, `PIN_RIGHT_STEP`, `PIN_RIGHT_DIR`, `PIN_ENABLE`.
- **Yön Tersleme**: `LEFT_DIR_INVERTED`, `RIGHT_DIR_INVERTED` teker yönünü düzeltmek için.
- **PID Kazançları**: `Kp`, `Ki`, `Kd` (çıktı birimi adım/sn). Önce Kp, sonra Kd, en son Ki ayarlayın.
- **Filtre**: `COMPLEMENTARY_ALPHA` (0.98 tipik). Daha yüksek -> gyro ağırlığı artar.
- **Döngü Hızı**: `LOOP_HZ` (200-500 Hz önerilir).
- **Güvenlik Limitleri**: `FALL_ANGLE_DEG`, `REENABLE_ANGLE_DEG`, `REENABLE_STABLE_CYCLES`.
- **Çıkış Limitleri**: `MAX_STEP_RATE_SPS`, `MIN_ACTIVE_RATE_SPS`.

### Çalışma Mantığı
- Başlangıçta gyro bias kalibrasyonu yapılır. Robot sabit durmalı.
- Robot dikliğe yakınsa kontrol otomatik enable olur. Aksi halde motorlar kapalı kalır.
- `FALL_ANGLE_DEG` aşılırsa güvenlik için motorlar kapanır; yeniden dik konuma gelince tekrar enable edilir.
- PID çıktısı her iki teker için aynı verilir (ileri-geri denge). Yaw ve hız komutu eklemek isterseniz sol/sağ karışımına offset ekleyebilirsiniz.

### PID Ayarı (Pratik Yol)
1. `Ki=0`, `Kd=0` ile başlayın.
2. `Kp`'yi yavaşça artırın (ör: 80 -> 120 -> 180 -> 240). Robot dik konumda titremeden dengeye yakın olmalı. Eğer salınım çok ise Kp azaltın.
3. `Kd` ekleyin (ör: 2 -> 4 -> 6). Salınımı söndürür; aşırı yüksek Kd tepkiyi yavaşlatır ve gürültüye duyarlı olur.
4. En son `Ki`'yı çok küçük başlayarak ekleyin (ör: 0.05 -> 0.1). Sürekli hatayı kapatır; fazla olursa integratör şişmesi yapar.

### Kalibrasyon İpuçları
- Güç verir vermez robotu hareket ettirmeyin; gyro bias toplama sürüyor (`GYRO_CALIBRATION_SAMPLES`).
- IMU yönelimi: Bu örnekte X ekseni pitch (ileri-geri eğim) olarak varsayıldı. Kart yönünüz farklıysa `estimateAngleDeg` içindeki `accAngle` hesaplamasını ayarlayın.

### Sık Karşılaşılan Sorunlar
- Motorlar ters dönüyor: `LEFT_DIR_INVERTED`/`RIGHT_DIR_INVERTED` değiştirin veya motor kablolarını çaprazlayın.
- Aşırı titreşim: `Kp` azaltın, `Kd` artırın, `COMPLEMENTARY_ALPHA` ayarlayın, DLPF'yi (`REG_CONFIG`) 0x03 civarında tutun.
- Motor kaçırıyor/ısı çok artıyor: Akım limitini doğru ayarlayın, mikroadım oranını düşürün, `MAX_STEP_RATE_SPS`'i sınırlayın.
- Hareket yok: EN pin seviyesi (A4988/DRV8825 için LOW aktif) ve GND referanslarını kontrol edin.

### Geliştirme (İleri)
- Hız/konum referansı için enkoder ekleyip dış döngü ile kapatabilirsiniz.
- Yaw kontrolü ve direksiyon için teker setpointlerine diferansiyel ekleme yapabilirsiniz.
- `Wire.setClock(400000)` ile I2C hızını 400kHz'e çıkarabilirsiniz (Nano destekler).

### Güvenlik
- LiPo ve motor akımları ile çalışırken uygun sigorta ve soğutma kullanın.
- Masa üstünde değil, boş bir alanda test edin. İlk denemelerde robotu elde hafif destekleyin.

