## Arduino Nano + MPU6050 + NEMA17 Kendini Dengeleyen Robot

Bu proje, Arduino Nano, MPU6050 ivme/gyro sensörü ve A4988/DRV8825 gibi STEP/DIR sürücüleri üzerinden sürülen NEMA17 step motorlar ile iki tekerlekli bir denge robotu içindir. Harici kütüphane gerekmez; yalnızca Wire (I2C) kullanılır.

### Donanım

- **Mikrodenetleyici**: Arduino Nano (ATmega328P, 16 MHz)
- **IMU**: MPU6050 (I2C)
- **Motorlar**: 2x NEMA17 step motor
- **Sürücüler**: 2x A4988 veya DRV8825 (STEP/DIR/EN)
- **Güç**: 12V/2–5A (mekaniğe bağlı), sürücü GND ile Arduino GND ortak olmalı

### Pin Bağlantıları (varsayılan)

- Sol motor: `STEP=D2`, `DIR=D3`
- Sağ motor: `STEP=D5`, `DIR=D6`
- Sürücü Enable (ortak): `EN=D8` (LOW=aktif)
- MPU6050: `SDA=A4`, `SCL=A5`, `VCC=5V`, `GND=GND`

> Not: Bir tekerin mekanik montajı ters ise yönler için `INVERT_DIR_LEFT/RIGHT` sabitlerini ayarlayın.

### Yazılımı Yükleme

1. Arduino IDE veya `arduino-cli` ile açın.
2. Kart: “Arduino Nano”. İşlemci: ATmega328P (Gerekirse Old Bootloader).
3. `arduino/self_balance/self_balance.ino` dosyasını derleyip yükleyin.

### Çalıştırma ve Kalibrasyon

1. Robotu elde düz ve sabit tutarak gücü verin. Açılışta ~1–2 sn gyro ofset kalibrasyonu yapılır. Seri portta “Calibrating…” mesajını görürsünüz.
2. Açı `RECOVER_ANGLE_DEG` içinde ise motorlar otomatik etkinleşir. Devrilme algılanırsa `FALL_ANGLE_DEG` üstünde motorlar kapatılır.

### Ayarlanabilir Sabitler

- `INVERT_DIR_LEFT/RIGHT`: Her teker için yön tersleme.
- `ENABLE_ACTIVE_LOW`: EN pin mantığı (A4988/DRV8825 için LOW=enable).
- `ANGLE_SIGN`: Kartın açı yönü ters ise `-1.0` yapın.
- `OUTPUT_SIGN`: Robot eğildiğinde ters yöne kaçıyorsa `-1.0` yapın.
- `MAX_STEPS_PER_SECOND`: Maksimum hız (steps/s). Mekaniğe göre arttırın.
- `MAX_ACCEL_STEPS_PER_SEC2`: İvmelenme limiti (steps/s^2).
- `FALL_ANGLE_DEG`, `RECOVER_ANGLE_DEG`: Güvenlik eşikleri.

### PID Tuning (Pratik Yol)

1. `Ki=0`, `Kd=0` yapıp yalnızca `Kp` ile başlayın. Robot hafifçe titreyene kadar `Kp`’yi artırın.
2. Salınımı azaltmak için `Kd` ekleyin (küçük adımlarla artırın).
3. Hafif ileri/geri sürünme kalıyorsa küçük bir `Ki` ekleyin.

Seri porttan anlık ayar:

- `w/s`: `Kp` +/− 1.0
- `e/d`: `Ki` +/− 0.05
- `r/f`: `Kd` +/− 0.05
- `y/t`: `angleTrim` +/− 0.2° (denge noktası ince ayar)
- `m`: Motorları aç/kapat
- `h`: Yardım

### Yön/İşaret Sorunlarını Çözme

- Robot öne eğildiğinde ileri gitmek yerine geri kaçıyorsa `OUTPUT_SIGN` işaretini ters çevirin.
- Açı değeri ters artıyorsa `ANGLE_SIGN` işaretini ters çevirin.
- Bir tekerin yönü diğerine göre ters ise ilgili `INVERT_DIR_*`’ı tersleyin.

### Mikrostep ve Hız

`MAX_STEPS_PER_SECOND` sınırı mikrostep ayarına, besleme gerilimine ve sürücü akımına bağlıdır. 1/16 mikrostep ve 12V ile birkaç bin steps/s tipiktir. Gerektiğinde arttırın ancak aşırı akımı ve sürücü ısınmasını kontrol edin.

### Güvenlik

- Besleme GND’si ile Arduino GND’sini mutlaka ortaklayın.
- İlk denemelerde robotu elde/askıda tutarak ayarlayın.
- `FALL_ANGLE_DEG` ile devrilme anında motorların kapanması sağlanır.

### Dosyalar

- `self_balance.ino`: Tüm kontrol kodu (MPU okuma, filtrasyon, PID, step/direction üretimi).

