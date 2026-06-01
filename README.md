# ITOBOTAutoRuntime

ITOBOTAutoRuntime, ITOBOT Auto Studio tarafından üretilen autonomous C++ header dosyalarındaki `AutoCommand[]` rutinlerini ESP32/Arduino firmware içinde çalıştıran küçük ve donanımdan bağımsız bir runtime kütüphanesidir.

Bu kütüphane motor sürücüleri, gyro, vision sistemi veya mekanizma donanımını doğrudan kontrol etmez. Robot firmware'i kendi mevcut fonksiyonlarını callback olarak runtime'a bağlar.

## Installation via ZIP

Arduino Library Manager submission gelene kadar en basit kurulum ZIP import ile yapılır:

1. GitHub repository sayfasından `Code -> Download ZIP` ile kütüphaneyi indirin.
2. Arduino IDE içinde `Sketch -> Include Library -> Add .ZIP Library...` menüsünü açın.
3. İndirilen ZIP dosyasını seçin.
4. Sketch içinde şu include'u kullanın:

```cpp
#include <ITOBOTAutoRuntime.h>
```

Alternatif olarak bu repository klasörünü Arduino `libraries` klasörünüze kopyalayabilirsiniz. Klasör adı `ITOBOTAutoRuntime` veya `itobot-auto-runtime` olabilir; önemli olan `library.properties` dosyasının klasör kökünde kalmasıdır.

## Arduino IDE Example Usage

Arduino IDE'de örnekleri açmak için:

1. `File -> Examples -> ITOBOTAutoRuntime -> BasicAutoRunner`
2. veya `File -> Examples -> ITOBOTAutoRuntime -> VisionPickupIntegration`
3. ESP32 board'unuzu seçin.
4. Sketch'i compile/upload edin.

`BasicAutoRunner` runtime state machine ve temel komut akışını gösterir. `VisionPickupIntegration` mevcut vision autonomous state machine'in `start/update/cancel` callback'leri ile nasıl bağlanacağını gösterir.

## Bu Kütüphane Ne Yapar?

- Auto Studio generated `AutoCommand[]` dizilerini sırayla çalıştırır.
- `DriveHoldYaw` ve `StrafeHoldYaw` komutlarında gyro yaw hold düzeltmesi üretir.
- `TurnToYaw` komutunu delay ile değil, gyro feedback ile hedef açıya gelene kadar çalıştırır.
- `Wait`, `IntakeOn`, `IntakeOff`, `Drop`, `Stop` komutlarını callback üzerinden yürütür.
- `VisionPickup` komutunu async olarak başlatır, her loop'ta vision update callback'ini çağırır ve vision bittiğinde sonraki komuta geçer.
- Komut timeout, error, cancel ve finish durumlarında drive motorlarını güvenli şekilde durdurmaya çalışır.

## Ne Yapmaz?

- Sensör okumaz.
- PWM veya motor çıkışı yazmaz.
- Encoder odometry veya trajectory following yapmaz.
- Robot-specific library'lere bağlı değildir.
- Hot path içinde heap allocation veya STL container kullanmaz.

## Auto Studio Generated `.h` Nasıl Include Edilir?

Yeni Auto Studio export'ları runtime ile uyumlu şekilde şu include'u üretir:

```cpp
#pragma once
#include <ITOBOTAutoRuntime.h>
```

Robot firmware tarafında runtime ve generated auto header birlikte include edilir:

```cpp
#include <ITOBOTAutoRuntime.h>
#include "generated/path_1_auto.h"
```

Generated header içinde genellikle şu semboller bulunur:

```cpp
static const AutoCommand PATH_1_AUTO[] = { ... };
static const size_t PATH_1_AUTO_COUNT = ...;

static const AutoEventInfo PATH_1_EVENTS[] = { ... };
static const size_t PATH_1_EVENTS_COUNT = ...;

static const AutoPathInfo PATH_1_INFO = { ... };
```

Birden fazla routine export edildiyse şu registry de bulunabilir:

```cpp
static const AutoRoutineRef PROJECT_AUTOS[] = { ... };
static const size_t PROJECT_AUTOS_COUNT = ...;
```

Runtime'ı başlatmak için:

```cpp
AutoRunner autoRunner;

void autonomousInit() {
  autoRunner.begin(PATH_1_AUTO, PATH_1_AUTO_COUNT);
}
```

## AutoRuntimeIO Callback'leri Nasıl Bağlanır?

Runtime motor veya mekanizma fonksiyonlarını doğrudan bilmez. Firmware bu fonksiyonları `AutoRuntimeIO` ile bağlar.

```cpp
AutoRunner autoRunner;
AutoRuntimeIO autoIO;
AutoRuntimeConfig autoConfig;

void runtimeDriveMecanum(float vx, float vy, float omega) {
  driveMecanum(vx, vy, omega);
}

void runtimeStopDrive() {
  stopMotors();
}

void runtimeSetIntake(float power) {
  setSingleMotorI2C(INTAKE, power);
}

void runtimeDrop() {
  stopMotors();
  setSingleMotorI2C(INTAKE, 0.0f);
  Serial.println("DROP command placeholder");
}

void runtimeError(const char* message) {
  Serial.print("Auto runtime error: ");
  Serial.println(message);
}

void configureRuntimeIo() {
  clearAutoRuntimeIO(autoIO);
  autoIO.driveMecanum = runtimeDriveMecanum;
  autoIO.stopDrive = runtimeStopDrive;
  autoIO.setIntake = runtimeSetIntake;
  autoIO.drop = runtimeDrop;
  autoIO.startVisionPickup = runtimeStartVisionPickup;
  autoIO.updateVisionPickup = runtimeUpdateVisionPickup;
  autoIO.cancelVisionPickup = runtimeCancelVisionPickup;
  autoIO.onCommandStart = runtimeCommandStart;
  autoIO.onCommandFinish = runtimeCommandFinish;
  autoIO.onError = runtimeError;
}
```

`driveMecanum(vx, vy, omega)` normalize edilmiş bir komut arayüzüdür:

- `vx`: ileri/geri güç, `[-1, 1]`
- `vy`: sağ/sol strafe güç, `[-1, 1]`
- `omega`: dönüş gücü, `[-1, 1]`

Pozitif `omega` robot yaw değerini artıracak yönde olmalıdır. Robotun dönüş işareti ters ise adapter içinde `omega` terslenmelidir.

Tank-like firmware kullanıyorsanız `vy` ignore edilebilir:

```cpp
void runtimeDriveMecanum(float vx, float vy, float omega) {
  (void)vy;
  setTankPower(vx - omega, vx + omega);
}
```

## Autonomous Loop Örneği

```cpp
void autonomousInit() {
  configureRuntimeIo();

  autoConfig.turnKp = 0.018f;
  autoConfig.turnToleranceDeg = 5.0f;
  autoConfig.maxTurnPower = 0.32f;
  autoConfig.minTurnPower = 0.18f;
  autoConfig.headingHoldKp = 0.014f;
  autoConfig.maxHeadingCorrection = 0.25f;
  autoConfig.commandTimeoutMs = 8000;
  autoConfig.stopOnTimeout = true;
  autoConfig.visionOwnsDrive = true;

  autoRunner.begin(PATH_1_AUTO, PATH_1_AUTO_COUNT, autoConfig);
}

void autonomousLoop() {
  updateGyro();

  AutoRuntimeInput input;
  input.nowMs = millis();
  input.yawDeg = currentYaw;
  input.visionPickupFinished = (visionState == VISION_DROP);

  autoRunner.update(input, autoIO);
}
```

`autonomousLoop()` içinde delay kullanılmamalıdır. Runtime komut sürelerini ve timeout'ları `input.nowMs` üzerinden takip eder.

## VisionPickup Update Callback Nasıl Çalışır?

`Cmd::VisionPickup` özel bir async komuttur.

Komut başladığında runtime:

1. `startVisionPickup()` callback'ini bir kez çağırır.
2. Vision komutu aktif olduğu sürece her `autoRunner.update()` tick'inde `updateVisionPickup()` callback'ini çağırır.
3. Sonra `input.visionPickupFinished` değerini kontrol eder.
4. `visionPickupFinished == true` olduğunda komutu bitirir ve sonraki AutoCommand'a geçer.
5. Timeout olursa `cancelVisionPickup()` çağırır, drive'ı durdurur ve config'e göre error veya skip davranışı uygular.

Örnek:

```cpp
void runtimeStartVisionPickup() {
  resetVisionTarget();
  setVisionState(VISION_SEARCH);
}

void runtimeUpdateVisionPickup() {
  runVisionAutonomy();
}

void runtimeCancelVisionPickup() {
  stopMotors();
  setSingleMotorI2C(INTAKE, 0.0f);
  resetVisionTarget();
}
```

Eğer mevcut `runVisionAutonomy()` fonksiyonunuz zaten `processVisionSerial()` çağırıyorsa, `autonomousLoop()` içinde ayrıca `processVisionSerial()` çağırmayın. Böylece aynı loop içinde vision serial iki kez parse edilmez.

`updateVisionPickup` opsiyoneldir. Null bırakılırsa runtime sadece `visionPickupFinished` flag'inin true olmasını bekler.

## BasicAutoRunner ve VisionPickupIntegration Examples Nasıl Compile Edilir?

Arduino CLI ve ESP32 core kurulumu:

```sh
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

PowerShell helper script:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/verify-arduino-esp32.ps1
```

Manuel compile:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries . examples/BasicAutoRunner
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries . examples/VisionPickupIntegration
```

`BasicAutoRunner` basit timed drive, turn, wait ve intake/drop komutlarını gösterir.

`VisionPickupIntegration` mevcut robot firmware'deki vision state machine'in runtime'a nasıl bağlanacağını gösterir:

- `runtimeStartVisionPickup()`
- `runtimeUpdateVisionPickup()`
- `runtimeCancelVisionPickup()`
- `visionPickupFinished` input flag'i

## Host Testleri

Windows PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run-host-tests.ps1
```

Linux/macOS:

```sh
sh scripts/run-host-tests.sh
```

Script `g++`, `clang++` veya MSVC `cl` arar, `test/AutoRunnerTests.cpp` dosyasını compile eder ve test executable'ını çalıştırır.

## No-Odometry / Timed+Gyro Limitation

Bu runtime encoder odometry veya gerçek trajectory following yapmaz. Auto Studio export'u final firmware tarafında timed+gyro approximations olarak çalışır.

Bu şu anlama gelir:

- Robot sahadaki gerçek pozisyonunu bilmez.
- Drive motor encoder feedback kullanılmaz.
- Mesafeler süreye, kalibrasyona ve güç ayarına bağlıdır.
- Gyro sadece heading hold ve `TurnToYaw` için kullanılır.
- Batarya voltajı, zemin sürtünmesi, teker kayması ve çarpışmalar gerçek yolu değiştirebilir.

Bu yüzden ilk testler her zaman düşük güçte ve robot güvenli şekilde yapılmalıdır:

1. Wheels lifted bench test.
2. Kısa forward smoke auto.
3. Küçük açılı `TurnToYaw`.
4. VisionPickup state machine testi.
5. Tam generated routine testi.
