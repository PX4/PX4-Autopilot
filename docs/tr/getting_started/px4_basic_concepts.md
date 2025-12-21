# Temel Kavramlar

Bu konu, dronlara ve PX4 kullanımına temel bir giriş sağlar (çoğunlukla acemi kullanıcılar içindir ancak daha deneyimli kullanıcılar için de iyi bir giriş niteliğindedir).

Temel kavramlara zaten aşinaysanız, kendi otopilot donanımınızın kablolamasını nasıl yapacağınızı öğrenmek için [Temel Montaj](../assembly/index.md) bölümüne geçebilirsiniz.
Firmware (bellenim) yüklemek ve aracı _QGroundControl_ ile kurmak için [Temel Yapılandırma](../config/index.md) bölümüne bakın.

## Drone Nedir?

Bir drone veya İnsansız Araç (İA), manuel veya otonom olarak kontrol edilebilen insansız bir "robotik" araçtır.
Havada, karada, suyun üzerinde/altında seyahat edebilirler ve hava fotoğrafçılığı/video, kargo taşıma, yarış, arama ve haritalama gibi birçok [tüketici, endüstriyel, hükümet ve askeri uygulamada](https://px4.io/ecosystem/commercial-systems/) kullanılırlar.

Dronlar daha resmi olarak İnsansız Hava Aracı (İHA - UAV), İnsansız Kara Aracı (İKA - UGV), İnsansız Su Üstü Aracı (İSA - USV), İnsansız Su Altı Aracı (İSA - UUV) olarak adlandırılır.

::: info
İnsansız Hava Sistemi (İHS - UAS) terimi tipik olarak bir İHA'yı ve bir yer kontrol istasyonu ve/veya radyo kumandası dahil olmak üzere tam bir sistemin diğer tüm bileşenlerini ve dronu kontrol etmek, verileri yakalamak ve işlemek için kullanılan diğer sistemleri ifade eder.
:::

## Drone Türleri

Birçok farklı araç gövdesi (türü) vardır ve türlerin içinde de birçok varyasyon bulunur.
Bazı türler ve en uygun oldukları kullanım durumları aşağıda listelenmiştir.

- [Çok Rotorlular (Multicopters)](../frames_multicopter/index.md) — Çok rotorlular, daha kısa ve genellikle daha yavaş uçuş pahasına hassas havada asılı kalma (hover) ve dikey kalkış sunar.
  En popüler uçan araç türüdürler, çünkü montajları kolaydır ve PX4'ün onları uçurmayı kolaylaştıran ve kamera platformu olarak çok uygun hale getiren modları vardır.
- [Helikopterler](../frames_helicopter/index.md) — Helikopterler, Çok Rotorlulara benzer avantajlara sahiptir ancak mekanik olarak daha karmaşık ve daha verimlidirler.
  Ayrıca uçurulmaları çok daha zordur.
- [Uçaklar (Sabit Kanat)](../frames_plane/index.md) — Sabit kanatlı araçlar, çok rotorlulardan daha uzun ve daha hızlı uçuş sunar, bu nedenle yer haritalama vb. için daha iyi kapsama alanı sağlarlar.
  Ancak uçurulmaları ve indirilmeleri çok rotorlulardan daha zordur ve havada asılı kalmanız veya çok yavaş uçmanız gerekiyorsa (örneğin dikey yapıları incelerken) uygun değildirler.
- [VTOL](../frames_vtol/index.md) (Dikey Kalkış ve İniş) - Hibrit Sabit Kanat/Çok Rotorlu araçlar her iki dünyanın da en iyisini sunar: dikey modda kalkış yapıp bir çok rotorlu gibi havada asılı kalabilirler, ancak daha fazla alan taramak için bir uçak gibi ileri uçuşa geçiş yapabilirler.
  VTOL'ler genellikle hem çok rotorlu hem de sabit kanatlı hava araçlarından daha pahalıdır ve yapımı ve ayarlanması daha zordur.
  Tiltrotorlar, tailsitterlar, quadplaneler vb. gibi çeşitli türleri vardır.
- [Hava Gemileri](../frames_airship/index.md)/[Balonlar](../frames_balloon/index.md) — Genellikle uçuş hızı ve yönü üzerinde sınırlı (veya hiç) kontrole sahip olma pahasına yüksek irtifa ve uzun süreli uçuş sunan havadan hafif araçlardır.
- [Roverlar (Kara Araçları)](../frames_rover/index.md) — Araba benzeri kara araçlarıdır.
  Kontrolleri basittir ve kullanımları genellikle eğlencelidir.
  Çoğu hava aracı kadar hızlı gidemezler, ancak daha ağır yükler taşıyabilirler ve dururken çok fazla güç harcamazlar.
- **Tekneler** — Su üstü araçları.
- [Denizaltılar](../frames_sub/index.md) — Su altı araçları.

Daha fazla bilgi için bkz:

- [Araç Türleri ve Kurulum](../airframes/index.md)
- [Gövde Kurulumu](../config/airframe.md)
- [Gövde Referansı](../airframes/airframe_reference.md).

## Otopilotlar

Dronun "beynine" otopilot denir.

En azından _uçuş kontrolcüsü_ (FC) donanımı üzerinde gerçek zamanlı bir işletim sisteminde ("RTOS") çalışan _uçuş yığını_ (flight stack) yazılımından oluşur.
Uçuş yığını, temel stabilizasyon ve güvenlik özelliklerini sağlar ve genellikle manuel uçuş için bir düzeyde pilot yardımı ve kalkış, iniş ve önceden tanımlanmış görevleri yürütme gibi yaygın görevleri otomatikleştirmeyi de içerir.

Bazı otopilotlar ayrıca "daha üst düzey" komuta ve kontrol sağlayabilen ve daha gelişmiş ağ, bilgisayar görüşü ve diğer özellikleri destekleyebilen genel amaçlı bir bilgi işlem sistemi içerir.
Bu, ayrı bir [eşlikçi bilgisayar (companion computer)](#offboard-companion-computer) olarak uygulanabilir, ancak gelecekte tamamen entegre bir bileşen olma ihtimali giderek artmaktadır.

## PX4 Uçuş Yığını (Flight Stack)

[PX4](https://px4.io/), NuttX RTOS üzerinde çalışan güçlü bir açık kaynak otopilot _uçuş yığınıdır_.

PX4'ün temel özelliklerinden bazıları şunlardır:

- [Çok rotorlular](../frames_multicopter/index.md), [sabit kanatlı uçaklar](../frames_plane/index.md), [VTOL'ler](../frames_vtol/index.md) (hibrit çok rotorlu/sabit kanatlı), [kara araçları](../frames_rover/index.md) ve [su altı araçları](../frames_sub/index.md) dahil olmak üzere birçok farklı araç gövdesini/türünü destekler.
- [Uçuş kontrolcüsü](#flight-controller), [sensörler](#sensors), [faydalı yükler](#payloads) ve diğer çevre birimleri için harika drone bileşeni seçenekleri sunar.
- Esnek ve güçlü [uçuş modları](#flight-modes) ve [güvenlik özellikleri](#safety-settings-failsafe).
- [Eşlikçi bilgisayarlar](#offboard-companion-computer) ve [ROS 2](../ros2/user_guide.md) ve [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) gibi [robotik API'leri](../robotics/index.md) ile sağlam ve derin entegrasyon.

PX4; [QGroundControl](#qgc) yer istasyonunu, [Pixhawk donanımını](https://pixhawk.org/) ve MAVLink protokolünü kullanarak eşlikçi bilgisayarlar, kameralar ve diğer donanımlarla entegrasyon için [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html)'yı içeren daha geniş bir drone platformunun temel bir parçasıdır.
PX4, [Dronecode Project](https://dronecode.org/) tarafından desteklenmektedir.

## Yer Kontrol İstasyonları (GCS)

Yer Kontrol İstasyonları (GCS), İA operatörlerinin bir dronu ve faydalı yüklerini izlemesini ve kontrol etmesini sağlayan yer tabanlı sistemlerdir.
PX4 ile çalıştığı bilinen ürünlerin bir alt kümesi aşağıda listelenmiştir.

### QGroundControl {#qgc}

Dronecode GCS yazılımının adı [QGroundControl](https://qgroundcontrol.com/) ("QGC")'dur.
Windows, Android, MacOS veya Linux donanımlarında çalışır ve çok çeşitli ekran form faktörlerini destekler.
[Buradan](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (ücretsiz olarak) indirebilirsiniz.

![QGC Ana Ekranı](../../assets/concepts/qgc_fly_view.png)

QGroundControl, gerçek zamanlı uçuş ve güvenlik bilgilerini almanızı ve işaretle-ve-tıkla arayüzünü kullanarak aracı, kamerayı ve diğer faydalı yükleri kontrol etmenizi sağlayan bir telemetri radyosu (iki yönlü veri bağlantısı) kullanarak dron ile iletişim kurar.
Bunları destekleyen donanımlarda, joystick kumandalarını kullanarak aracı manuel olarak da uçurabilirsiniz.
QGC ayrıca otonom görevleri görsel olarak planlamak, yürütmek ve izlemek, coğrafi çitler (geofence) ayarlamak ve çok daha fazlası için kullanılabilir.

QGroundControl masaüstü sürümleri ayrıca PX4 bellenimini (firmware) yüklemek (flashlamak) ve dronun otopilot/uçuş kontrol donanımında PX4'ü yapılandırmak için kullanılır.

### Auterion Mission Control (AMC) {#amc}

[Auterion Mission Control](https://auterion.com/product/mission-control/), araç yapılandırmasından ziyade _pilotlar_ için optimize edilmiş güçlü ve tam özellikli bir yer kontrol istasyonu uygulamasıdır.
Auterion ürünleriyle çalışmak üzere tasarlanmış olsa da, "saf" (vanilla) PX4 ile de kullanılabilir.

Daha fazla bilgi için bkz:

- [AMC dokümanları](https://docs.auterion.com/vehicle-operation/auterion-mission-control)
- [Auterion Suite'ten İndir](https://suite.auterion.com/)

## Drone Bileşenleri ve Parçaları

### Uçuş Kontrolcüsü (Flight Controller)

Uçuş kontrolcüleri (FC), PX4 uçuş yığını belleniminin yüklendiği ve çalıştırıldığı donanımlardır.
PX4'ün durumunu belirlediği sensörlere ve aracı stabilize etmek ve hareket ettirmek için kullandığı aktüatörlere/motorlara bağlanırlar.

<img src="../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x.jpg" width="230px" title="CUAV Pixhawk 6X" >

PX4, [Pixhawk Serisi](../flight_controller/pixhawk_series.md) kontrolcülerden Linux bilgisayarlara kadar birçok farklı türde [Uçuş Kontrol Donanımında](../flight_controller/index.md) çalışabilir.
Bunlar arasında [Pixhawk Standart](../flight_controller/autopilot_pixhawk_standard.md) ve [üretici destekli](../flight_controller/autopilot_manufacturer_supported.md) kartlar bulunur.
Aracınızın fiziksel kısıtlamalarına, gerçekleştirmek istediğiniz faaliyetlere ve maliyete uygun bir kart seçmelisiniz.

Daha fazla bilgi için bkz: [Uçuş Kontrolcüsü Seçimi](flight_controller_selection.md)

### Sensörler

PX4, aracı stabilize etmek ve otonom kontrolü sağlamak için ihtiyaç duyduğu araç durumunu belirlemek üzere sensörleri kullanır.
Araç durumları şunları içerir: konum/irtifa, istikamet (heading), hız, hava hızı, yönelim (attitude), farklı eksenlerdeki dönüş oranları, pil seviyesi vb.

PX4 _asgari olarak_ bir [jiroskop](../sensor/gyroscope.md), [ivmeölçer](../sensor/accelerometer.md), [manyetometre](../gps_compass/magnetometer.md) (pusula) ve [barometre](../sensor/barometer.md) gerektirir.
Bu asgari sensör seti, [Pixhawk Serisi](../flight_controller/pixhawk_series.md) uçuş kontrolcülerine entegre edilmiştir (ve diğer kontrolcü platformlarında da bulunabilir).

Kontrolcüye ek/harici sensörler takılabilir.
Aşağıdaki sensörler önerilir:

- Tüm otomatik modları ve bazı manuel/destekli modları etkinleştirmek için bir [GNSS/GPS](../gps_compass/index.md) veya başka bir küresel konum kaynağına ihtiyaç vardır.

  Genellikle bir GNSS ve Pusulayı birleştiren bir modül kullanılır, çünkü harici bir pusula, uçuş kontrolcüsündeki dahili pusuladan elektromanyetik girişime karşı daha az duyarlı hale getirilebilir.

- [Hava hızı sensörleri](../sensor/airspeed.md), sabit kanatlı ve VTOL araçlar için şiddetle tavsiye edilir.
- [Mesafe Sensörleri \(Rangefinders\)](../sensor/rangefinders.md), tüm araç türleri için şiddetle tavsiye edilir, çünkü daha yumuşak ve daha sağlam inişler sağlarlar ve çok rotorlular üzerinde arazi takibi gibi özellikleri mümkün kılarlar.
- [Optik Akış (Optical Flow) Sensörleri](../sensor/optical_flow.md), GNSS olmayan ortamlarda navigasyonu desteklemek için çok rotorlular ve VTOL üzerinde mesafe sensörleriyle birlikte kullanılabilir.

Sensörler hakkında daha fazla bilgi için bkz: [Sensör Donanımı ve Kurulumu](../sensor/index.md).

### Çıkışlar: Motorlar, Servolar, Eyleyiciler

PX4 _çıkışları_ şunları kontrol etmek için kullanır: motor hızı (örn. [ESC](#escs-motors) aracılığıyla), kanatçıklar (aileron) ve flaplar gibi uçuş yüzeyleri, kamera tetikleyicileri, paraşütler, tutucular (grippers) ve diğer birçok faydalı yük türü.

Çıkışlar PWM portları veya DroneCAN düğümleri (örn. DroneCAN [motor kontrolcüleri](../dronecan/escs.md)) olabilir.
Aşağıdaki resimler [Pixhawk 4](../flight_controller/pixhawk4.md) ve [Pixhawk 4 mini](../flight_controller/pixhawk4_mini.md) için PWM çıkış portlarını göstermektedir.

![Pixhawk 4 çıkış portları](../../assets/flight_controller/pixhawk4/pixhawk4_main_aux_ports.jpg) ![Pixhawk4 mini MAIN portları](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_pwm.png)

Çıkışlar `MAIN` ve `AUX` çıkışları olarak ikiye ayrılır ve ayrı ayrı numaralandırılır (yani `MAINn` ve `AUXn`, burada `n` 1'den genellikle 6 veya 8'e kadardır).
Ayrıca `IO PWM Out` ve `FMU PWM OUT` (veya benzeri) olarak da işaretlenmiş olabilirler.

:::warning
Bir uçuş kontrolcüsü yalnızca `MAIN` PWM çıkışlarına sahip olabilir (örneğin _Pixhawk 4 Mini_ gibi) veya `MAIN` veya `AUX` üzerinde yalnızca 6 çıkışa sahip olabilir.
[Gövdeniz](../airframes/airframe_reference.md) için yeterli porta/çıkışa sahip bir kontrolcü seçtiğinizden emin olun.
:::

QGroundControl'de ilgili işlevi ("Motor 1") istenen çıkışa ("AUX1") atayarak hemen hemen her çıkışı herhangi bir motora veya diğer eyleyiciye bağlayabilirsiniz: [Eyleyici Yapılandırması ve Testi](../config/actuators.md).
Her gövde için işlevlerin (motor ve kontrol yüzeyi eyleyici konumlarının) [Gövde Referansında](../airframes/airframe_reference.md) verildiğini unutmayın.

**Notlar:**

- Pixhawk kontrolcülerinde bir FMU kartı bulunur ve ayrı bir IO kartı _olabilir_.
  Bir IO kartı varsa, `AUX` portları doğrudan FMU'ya ve `MAIN` portları IO kartına bağlanır.
  Aksi takdirde `MAIN` portları FMU'ya bağlanır ve `AUX` portları yoktur.
- FMU çıkış portları, çok daha düşük gecikmeli davranış sağlayan [D-shot](../peripherals/dshot.md) veya _One-shot_ protokollerini (PWM'in yanı sıra) kullanabilir.
  Bu, yarışçılar ve daha iyi performans gerektiren diğer gövdeler için yararlı olabilir.
- `MAIN` ve `AUX` içinde yalnızca 6-8 çıkış vardır, çünkü çoğu uçuş kontrolcüsü yalnızca bu kadar PWM/Dshot/Oneshot çıkışına sahiptir.
  Teorik olarak veri yolu destekliyorsa çok daha fazla çıkış olabilir (yani bir UAVCAN veri yolu bu kadar az düğümle sınırlı değildir).

### ESC'ler ve Motorlar

Birçok PX4 dronu, bir Elektronik Hız Kontrolcüsü (ESC) aracılığıyla uçuş kontrolcüsü tarafından sürülen fırçasız motorlar kullanır.
(ESC, uçuş kontrolcüsünden gelen bir sinyali motora iletilen uygun bir güç seviyesine dönüştürür).

PX4 tarafından hangi ESC/Motorların desteklendiği hakkında bilgi için bkz:

- [ESC ve Motorlar](../peripherals/esc_motors.md)
- [ESC Kalibrasyonu](../advanced_config/esc_calibration.md)
- [ESC Firmware ve Protokollerine Genel Bakış](https://oscarliang.com/esc-firmware-protocols/) (oscarliang.com)

### Pil/Güç

PX4 dronları çoğunlukla Lityum-Polimer (LiPo) pillerden güç alır.
Pil genellikle sisteme bir [Güç Modülü](../power_module/index.md) veya _Güç Yönetim Kartı_ kullanılarak bağlanır; bu, uçuş kontrolcüsü ve (motorlar için) ESC'lere ayrı güç sağlar.

Piller ve pil yapılandırması hakkında bilgi [Pil Tahmini Ayarlama](../config/battery.md) bölümünde ve [Temel Montaj](../assembly/index.md) kılavuzlarında (örn. [Pixhawk 4 Kablolama Hızlı Başlangıç > Güç](../assembly/quick_start_pixhawk4.md#power)) bulunabilir.

### Manuel Kontrol

Pilotlar, bir [Radyo Kontrol (RC) Sistemi](../getting_started/rc_transmitter_receiver.md) veya QGroundControl aracılığıyla bağlanan bir [Joystick/Gamepad](../config/joystick.md) kontrolcüsü kullanarak aracı manuel olarak kontrol edebilir.

![Taranis X9D Verici](../../assets/hardware/transmitters/frsky_taranis_x9d_transmitter.jpg) <img src="../../assets/peripherals/joystick/micronav.jpg" alt="Photo of MicroNav, a ground controller with integrated joysticks" width="400px">

RC sistemleri, kontrol bilgilerini göndermek için özel bir yer tabanlı radyo vericisi ve araç tabanlı alıcı kullanır.
Yeni bir gövde tasarımını ilk kez ayarlarken/test ederken veya yarış/akrobasi uçuşları yaparken (ve düşük gecikmenin önemli olduğu diğer durumlarda) her zaman kullanılmalıdırlar.

Joystick sistemleri, "standart" bir bilgisayar oyun joystick'inden gelen kontrol bilgilerini MAVLink mesajlarına kodlamak ve (paylaşılan) telemetri radyo kanalını kullanarak araca göndermek için QGroundControl'ü kullanır.
Telemetri kanalınız yeterince yüksek bir bant genişliğine/düşük gecikmeye sahip olması koşuluyla, kalkış, haritalama vb. gibi çoğu manuel uçuş kullanım durumu için kullanılabilirler.

Joystickler genellikle entegre GCS/manuel kontrol sistemlerinde kullanılır çünkü bir joystick'i entegre etmek ayrı bir radyo sisteminden daha ucuz ve daha kolaydır ve çoğu kullanım durumu için düşük gecikme önemli değildir.
Ayrıca PX4 simülatörünü uçurmak için de mükemmeldirler, çünkü onları doğrudan yer kontrol bilgisayarınıza takabilirsiniz.

::: info
PX4, otonom uçuş modları için bir manuel kontrol sistemi _gerektirmez_.
:::

### Güvenlik Anahtarı (Safety Switch)

Araçlar, araç [arm edilmeden](#arming-and-disarming) (arm edildiğinde motorlara güç verilir ve pervaneler dönebilir) önce basılması gereken bir _güvenlik anahtarı_ içerebilir.

Bu anahtar neredeyse her zaman Pixhawk `GPS1` portuna bağlanan [GPS](../gps_compass/index.md) modülüne entegre edilmiştir — [buzzer](#buzzer) ve [UI LED](#leds) ile birlikte.

Bu anahtar varsayılan olarak devre dışı olabilir, ancak bu durum belirli uçuş kontrolcüsü ve gövde yapılandırmasına bağlıdır.
Anahtarın kullanımını [CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY) parametresi ile devre dışı bırakabilir/etkinleştirebilirsiniz.

::: info
Güvenlik anahtarları isteğe bağlıdır.
Birçok kişi, kullanıcıların bu kilidi etkinleştirmek/devre dışı bırakmak için bile olsa güç verilmiş bir sisteme asla yaklaşmamasının daha güvenli olduğunu savunur.
:::

### Buzzer (Sesli İkaz)

Araçlar genellikle araç durumunu ve uçuşa hazır olup olmadığını sesli olarak bildirmek için bir buzzer içerir (bkz. [Melodi anlamları](../getting_started/tunes.md)).

Bu buzzer neredeyse her zaman Pixhawk `GPS1` portuna bağlanan [GPS](../gps_compass/index.md) modülüne entegre edilmiştir — [güvenlik anahtarı](#safety-switch) ve [UI LED](#leds) ile birlikte.
Bildirim melodilerini [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER) parametresini kullanarak devre dışı bırakabilirsiniz.

### LED'ler

Araçlar, mevcut uçuşa hazır olma durumunu gösteren süper parlak bir [UI RGB LED](../getting_started/led_meanings.md#ui-led)'e sahip olmalıdır.

Tarihsel olarak bu, uçuş kontrol kartına dahil edilirdi.
Daha yeni uçuş kontrolcülerinde bu neredeyse her zaman Pixhawk `GPS1` portuna bağlanan [GPS](../gps_compass/index.md) modülüne entegre edilmiş bir [I2C çevresel birimidir](../sensor_bus/i2c_general.md) — [güvenlik anahtarı](#safety-switch) ve [buzzer](#buzzer) ile birlikte.

### Veri/Telemetri Radyoları

[Veri/Telemetri Radyoları](../telemetry/index.md), _QGroundControl_ gibi bir yer kontrol istasyonu ile PX4 çalıştıran bir araç arasında kablosuz bir MAVLink bağlantısı sağlayabilir.
Bu, bir araç uçuş halindeyken parametreleri ayarlamayı, telemetriyi gerçek zamanlı olarak incelemeyi, uçuş sırasında bir görevi değiştirmeyi vb. mümkün kılar.

### Offboard/Eşlikçi Bilgisayar (Companion Computer)

Bir [Eşlikçi Bilgisayar](../companion_computer/index.md) ("görev bilgisayarı" veya "offboard bilgisayar" olarak da adlandırılır), daha üst düzey komuta ve kontrol sağlamak için PX4 ile iletişim kuran ayrı bir araç üstü bilgisayardır.

Eşlikçi bilgisayar genellikle Linux çalıştırır, çünkü bu "genel" yazılım geliştirme için çok daha iyi bir platformdur ve dronların bilgisayar görüşü, ağ oluşturma vb. için önceden var olan yazılımlardan yararlanmasını sağlar.

Uçuş kontrolcüsü ve eşlikçi bilgisayar, donanım geliştirmeyi basitleştirmek için tek bir taban kartına önceden entegre edilebilir veya ayrı olabilir ve bir seri kablo, Ethernet kablosu veya wifi ile bağlanır.
Eşlikçi bilgisayar tipik olarak PX4 ile [MAVSDK](https://mavsdk.mavlink.io/) veya [ROS 2](../ros2/user_guide.md) gibi üst düzey bir Robotik API kullanarak iletişim kurar.

İlgili konular şunlardır:

- [Eşlikçi Bilgisayarlar](../companion_computer/index.md)
- [Offboard Modu](../flight_modes/offboard.md) - Bir GCS veya eşlikçi bilgisayardan PX4'ün offboard kontrolü için uçuş modu.
- [Robotik API'leri](../robotics/index.md)

### SD Kartlar (Çıkarılabilir Bellek)

PX4, [uçuş loglarını](../getting_started/flight_reporting.md) saklamak için SD hafıza kartlarını kullanır ve ayrıca UAVCAN çevre birimlerini kullanmak ve [görevleri](../flying/missions.md) uçurmak için de gereklidir.

Varsayılan olarak, SD kart yoksa PX4 önyükleme sırasında [biçimlendirme başarısız (2 bip)](../getting_started/tunes.md#format-failed) melodisini iki kez çalar (ve yukarıdaki özelliklerin hiçbiri kullanılamaz).

::: tip
Pixhawk kartlarında desteklenen maksimum SD kart boyutu 32GB'dır.
_SanDisk Extreme U3 32GB_ ve _Samsung EVO Plus 32_ [şiddetle tavsiye edilir](../dev_log/logging.md#sd-cards).
:::

SD kartlar yine de isteğe bağlıdır.
SD Kart yuvası içermeyen uçuş kontrolcüleri şunları yapabilir:

- Bildirim bipleri [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER) parametresi kullanılarak devre dışı bırakılır.
- [Logları](../dev_log/logging.md#log-streaming) başka bir bileşene (eşlikçi) aktarır.
- Görevleri RAM/FLASH'ta saklar.

## Faydalı Yükler (Payloads)

Faydalı yükler; haritalama görevlerindeki kameralar, radyasyon dedektörleri gibi denetimlerde kullanılan aletler ve teslim edilmesi gereken kargolar gibi kullanıcı veya görev hedeflerini karşılamak için araç tarafından taşınan ekipmanlardır.
PX4 birçok kamerayı ve çok çeşitli faydalı yükleri destekler.

Faydalı yükler [Uçuş Kontrolcüsü çıkışlarına](#outputs-motors-servos-actuators) bağlanır ve görevlerde otomatik olarak veya bir RC Kumanda veya Joystick'ten veya bir Yer İstasyonundan (MAVLink/MAVSDK komutları aracılığıyla) manuel olarak tetiklenebilir.

Daha fazla bilgi için bkz: [Faydalı Yükler ve Kameralar](../payloads/index.md)

## Arm ve Disarm İşlemi (Motorları Başlatma ve Kapatma)

Tüm motorlara ve eyleyicilere güç verildiğinde bir aracın _armed_ (kurulu/etkin), hiçbir şeye güç verilmediğinde ise _disarmed_ (devre dışı) olduğu söylenir.
Ayrıca, yalnızca servo eyleyicilere güç verildiği bir _prearmed_ (ön-kurulum) durumu da vardır, bu öncelikle test için kullanılır.

Bir araç genellikle yerdeyken disarmed durumdadır ve mevcut uçuş modunda kalkış yapmadan önce arm edilmelidir.

:::warning
Arm edilmiş araçlar tehlikelidir çünkü pervaneler daha fazla kullanıcı girişi olmadan herhangi bir zamanda dönmeye başlayabilir ve çoğu durumda hemen dönmeye başlayacaktır.
:::

Arm ve disarm işlemleri varsayılan olarak RC çubuk _hareketleri_ kullanılarak tetiklenir.
Mod 2 vericilerde, arm etmek için RC gaz/yaw çubuğunu (sol çubuk) bir saniye boyunca _sağ altta_ tutarsınız ve disarm etmek için çubuğu bir saniye boyunca sol altta tutarsınız.
Alternatif olarak PX4'ü bir RC anahtarı veya düğmesi kullanarak arm edecek şekilde yapılandırmak mümkündür (ve arm etme MAVLink komutları bir yer istasyonundan da gönderilebilir).

Kazaları azaltmak için, araç yerdeyken mümkün olduğunca az arm edilmiş halde tutulmalıdır.
Varsayılan olarak araçlar:

- Kullanılmadığında _Disarmed_ veya _Prearmed_ (motorlara güç verilmez) durumdadır ve kalkıştan önce açıkça _armed_ durumuna getirilmelidir.
- Arm edildikten sonra araç yeterince hızlı kalkış yapmazsa otomatik olarak disarm/prearm olur (disarm süresi yapılandırılabilir).
- İnişten kısa bir süre sonra otomatik olarak disarm/prearm olur (süre yapılandırılabilir).
- Araç "sağlıklı" bir durumda değilse arm işlemi engellenir.
- Araçta devreye sokulmamış bir [güvenlik anahtarı](#safety-switch) varsa arm işlemi engellenir.
- Bir VTOL aracı sabit kanat modundaysa arm işlemi engellenir ([varsayılan olarak](../advanced_config/parameter_reference.md#CBRK_VTOLARMING)).
- Arm işlemi, düşük pil gibi bir dizi başka isteğe bağlı [arm ön koşulu ayarı](../config/safety.md#arming-pre-conditions) nedeniyle engellenebilir.

Prearmed durumundayken eyleyicileri (servoları) kullanabilirsiniz, disarm ise her şeyin gücünü keser.
Prearmed ve disarmed durumlarının ikisi de güvenli olmalıdır ve belirli bir araç bunlardan birini veya her ikisini de destekleyebilir.

:::tip
Bazen bir araç bariz olmayan nedenlerle arm olmayabilir.
QGC v4.2.0 (yazım sırasındaki Günlük sürüm) ve sonrası, [Fly View > Arming and Preflight Checks](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#arm) bölümünde bir arm kontrol raporu sağlar.
PX4 v1.14'ten itibaren bu, arm sorunları hakkında olası çözümlerle birlikte kapsamlı bilgi sağlar.
:::

Arm ve disarm yapılandırmasının ayrıntılı bir özeti burada bulunabilir: [Prearm, Arm, Disarm Yapılandırması](../advanced_config/prearm_arm_disarm.md).

## Uçuş Modları

Modlar, kullanıcıya (pilota) farklı türde/seviyelerde araç otomasyonu ve otopilot yardımı sağlayan özel operasyonel durumlardır.

_Otonom modlar_ tamamen otopilot tarafından kontrol edilir ve pilot/uzaktan kumanda girişi gerektirmez.
Bunlar, örneğin kalkış, ana konuma dönüş ve iniş gibi yaygın görevleri otomatikleştirmek için kullanılır.
Diğer otonom modlar önceden programlanmış görevleri yürütür, bir GPS işaretini (beacon) takip eder veya bir offboard bilgisayardan veya yer istasyonundan gelen komutları kabul eder.

_Manuel modlar_, otopilotun yardımıyla kullanıcı tarafından (RC kontrol çubukları/joystick aracılığıyla) kontrol edilir.
Farklı manuel modlar farklı uçuş özelliklerini mümkün kılar - örneğin, bazı modlar akrobatik hareketlere izin verirken, diğerlerinde takla atmak imkansızdır ve araç rüzgara karşı konumunu/rotasını korur.

:::tip
Tüm modlar tüm araç türlerinde mevcut değildir ve bazı modlar yalnızca belirli koşullar karşılandığında kullanılabilir (örneğin, birçok mod küresel bir konum tahmini gerektirir).
:::

Her araç için PX4 içinde uygulanan uçuş modlarına genel bir bakış aşağıda bulunabilir:

- [Uçuş Modları (Çok Rotorlu)](../flight_modes_mc/index.md)
- [Uçuş Modları (Sabit Kanat)](../flight_modes_fw/index.md)
- [Uçuş Modları (VTOL)](../flight_modes_vtol/index.md)
- [Sürüş Modları (Rover)](../flight_modes_rover/index.md)

Farklı uçuş modlarını etkinleştirmek için uzaktan kumanda anahtarlarınızı nasıl ayarlayacağınıza dair talimatlar [Uçuş Modu Yapılandırması](../config/flight_mode.md) bölümünde verilmiştir.

PX4 ayrıca [PX4 ROS 2 Kontrol Arayüzü](../ros2/px4_ros2_control_interface.md) kullanılarak [ROS 2](../ros2/index.md)'de uygulanan harici modları da destekler.
Bunlar PX4 dahili modlarından ayırt edilemez ve dahili modları daha gelişmiş bir sürümle geçersiz kılmak veya tamamen yeni işlevler oluşturmak için kullanılabilir.
Bunların ROS 2'ye bağlı olduğunu ve bu nedenle yalnızca bir [eşlikçi bilgisayara](#offboard-companion-computer) sahip sistemlerde çalışabileceğini unutmayın.

## Güvenlik Ayarları (Failsafe / Arıza Koruma)

PX4, bir şeyler ters giderse aracınızı korumak ve kurtarmak için yapılandırılabilir failsafe sistemlerine sahiptir!
Bunlar, güvenli bir şekilde uçabileceğiniz alanları ve koşulları ve bir failsafe tetiklenirse gerçekleştirilecek eylemi (örneğin, iniş, konumu koruma veya belirtilen bir noktaya dönüş) belirlemenize olanak tanır.

::: info
Yalnızca _ilk_ failsafe olayı için eylemi belirleyebilirsiniz.
Bir failsafe meydana geldiğinde sistem özel bir işleme koduna girer, böylece sonraki failsafe tetikleyicileri ayrı sistem seviyesi ve araca özel kod tarafından yönetilir.
:::

Ana failsafe alanları aşağıda listelenmiştir:

- Düşük Pil
- Uzaktan Kumanda (RC) Kaybı
- Konum Kaybı (küresel konum tahmini kalitesi çok düşük).
- Offboard Kaybı (ör. eşlikçi bilgisayara bağlantı kaybı)
- Veri Bağlantısı Kaybı (ör. GCS'ye telemetri bağlantısı kaybı).
- Coğrafi Çit (Geofence) İhlali (aracı sanal bir silindir içinde uçuşla sınırlandırma).
- Görev Failsafe'i (önceki bir görevin yeni bir kalkış konumunda çalıştırılmasını önleme).
- Trafik kaçınma (ör. ADSB transponderlarından gelen transponder verileriyle tetiklenir).

Daha fazla bilgi için bkz: [Güvenlik](../config/safety.md) (Temel Yapılandırma).

## İstikamet ve Yönler (Heading)

Tüm araçların, teknelerin ve hava araçlarının ileri hareketlerine dayalı bir istikamet yönü veya yönelimi vardır.

![Gövde İstikameti](../../assets/concepts/frame_heading.png)

::: info
Bir VTOL Tailsitter için istikamet, çok rotorlu konfigürasyona göredir (yani kalkış, havada asılı kalma, iniş sırasındaki araç duruşu).
:::

Otopilotu aracın hareket vektörüyle hizalamak için aracın istikamet yönünü bilmek önemlidir.
Çok rotorluların her taraftan simetrik olsalar bile bir istikameti vardır!
Genellikle üreticiler istikameti belirtmek için renkli pervaneler veya renkli kollar kullanırlar.

![Gövde İstikameti ÜST](../../assets/concepts/frame_heading_top.png)

Çizimlerimizde çok rotorlunun ön pervaneleri için istikameti göstermek amacıyla kırmızı renk kullanacağız.

İstikamet hakkında derinlemesine bilgiyi [Uçuş Kontrolcüsü Yönelimi](../config/flight_controller_orientation.md) bölümünde okuyabilirsiniz.