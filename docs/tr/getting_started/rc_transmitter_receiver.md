# Radyo Kontrol (RC) Sistemleri

Bir Radyo Kontrol (RC) sistemi, aracınızı elde taşınan bir RC kontrolcüsü (kumanda) ile _manuel_ olarak kontrol etmek için kullanılabilir.
Bu konu, RC'nin nasıl çalıştığına, aracınız için uygun bir radyo sisteminin nasıl seçileceğine ve uçuş kontrolcüsüne nasıl bağlanacağına dair genel bir bakış sağlar.

:::tip
PX4 ayrıca bir [Joystick](../config/joystick.md) veya gamepad benzeri bir kontrolcü kullanılarak da manuel olarak kontrol edilebilir: bu bir RC sisteminden farklıdır!
[COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) parametresi, RC'nin mi (varsayılan), Joystick'in mi, her ikisinin mi yoksa hiçbirinin mi etkinleştirileceğini seçmek için [ayarlanabilir](../advanced_config/parameters.md).
:::

::: info
PX4, otonom uçuş modları için bir uzaktan kontrol sistemi gerektirmez.
:::

## RC Sistemleri Nasıl Çalışır?

Bir _RC sistemi_, operatör tarafından araca komut vermek için kullanılan yer tabanlı bir _uzaktan kontrol ünitesine_ (kumanda) sahiptir.
Kumanda, araç hareketini (örn. hız, yön, gaz/throttle, yaw, pitch, roll vb.) belirtmek ve otopilot [uçuş modlarını](../flight_modes/index.md) (örn. kalkış, iniş, kalkış yerine dönüş, görev vb.) etkinleştirmek için kullanılabilen fiziksel kontrollere sahiptir.
_Telemetri özellikli_ RC sistemlerinde, uzaktan kontrol ünitesi araçtan pil seviyesi, uçuş modu ve uyarılar gibi bilgileri de alabilir ve görüntüleyebilir.

![Taranis X9D Verici](../../assets/hardware/transmitters/frsky_taranis_x9d_transmitter.jpg)

Yer tabanlı RC kontrolcüsü, araç üzerindeki (uyumlu) bir radyo modülü ile eşleşen ve iletişim kuran bir radyo modülü içerir.
Araç tabanlı ünite (alıcı), uçuş kontrolcüsüne bağlanır.
Uçuş kontrolcüsü, mevcut otopilot uçuş moduna ve araç durumuna göre komutların nasıl yorumlanacağını belirler ve araç motorlarını ve eyleyicilerini uygun şekilde sürer.

::: info
Yer ve araç tabanlı radyo modülleri sırasıyla verici (transmitter) ve alıcı (receiver) olarak adlandırılır (çift yönlü iletişimi destekleseler bile) ve topluca bir _verici/alıcı çifti_ olarak anılırlar.
RC kontrolcüsü ve içerdiği radyo modülü genellikle "verici" veya "kumanda" olarak adlandırılır.
:::

Bir RC sisteminin önemli bir niteliği, kaç "kanalı" desteklediğidir.
Kanal sayısı, kumanda üzerindeki kaç farklı fiziksel kontrolün araca komut göndermek için kullanılabileceğini tanımlar (örn. kaç anahtar, kadran, kontrol çubuğu gerçekten kullanılabilir).

Bir hava aracı en az 4 kanalı destekleyen bir sistem kullanmalıdır (roll, pitch, yaw, thrust/gaz için).
Kara araçları en az iki kanala ihtiyaç duyar (direksiyon + gaz). 8 veya 16 kanallı bir verici, diğer mekanizmaları kontrol etmek veya otopilot tarafından sağlanan farklı [uçuş modlarını](../flight_modes/index.md) etkinleştirmek için kullanılabilecek ek kanallar sağlar.

## Uzaktan Kontrol Ünitesi (Kumanda) Türleri

<a id="transmitter_modes"></a>

### Hava Araçları için Uzaktan Kontrol Üniteleri

İHA'lar için en popüler uzaktan kontrol ünitesi _formu_ aşağıda gösterilmiştir.
Gösterildiği gibi roll/pitch ve throttle/yaw kontrolü için ayrı kontrol çubuklarına (stick) sahiptir (yani hava araçları en az 4 kanala ihtiyaç duyar).

![RC Temel Komutlar](../../assets/flying/rc_basic_commands.png)

Kontrol çubukları, anahtarlar vb. için çok sayıda olası düzen (layout) vardır.
Daha yaygın düzenlere belirli "Mod" numaraları verilmiştir. _Mod 1_ ve _Mod 2_ (aşağıda gösterilmiştir) yalnızca gazın (throttle) yerleşiminde farklılık gösterir.

![Mod1-Mod2](../../assets/concepts/mode1_mode2.png)

::: info
Mod seçimi büyük ölçüde bir zevk meselesidir (_Mod 2_ daha popülerdir).
:::

## Kara Araçları için Uzaktan Kontrol Üniteleri

Bir İnsansız Kara Aracı (İKA - UGV)/araba, direksiyon ve hız değerlerini göndermek için minimum 2 kanallı bir verici gerektirir.
Yaygın vericiler bu değerleri bir tekerlek ve tetik, iki tek eksenli kontrol çubuğu veya tek bir çift eksenli kontrol çubuğu kullanarak ayarlar.

Daha fazla kanal/kontrol mekanizması kullanmanıza engel hiçbir şey yoktur ve bunlar ek eyleyicileri ve otopilot modlarını devreye sokmak için çok yararlı olabilir.

## RC Sistemi Bileşenlerini Seçme

Birbiriyle uyumlu bir verici/alıcı çifti seçmeniz gerekecektir.
Ek olarak, alıcılar [PX4 ile](#compatible_receivers) ve uçuş kontrol donanımı ile uyumlu olmalıdır.

Uyumlu radyo sistemleri genellikle birlikte satılır.
Örneğin, [FrSky Taranis X9D ve FrSky X8R](https://hobbyking.com/en_us/frsky-2-4ghz-accst-taranis-x9d-plus-and-x8r-combo-digital-telemetry-radio-system-mode-2.html?___store=en_us) popüler bir kombinasyondur.

### Verici/Alıcı Çiftleri

En popüler RC ünitelerinden biri _FrSky Taranis X9D_'dir.
Dahili bir verici modülüne sahiptir ve önerilen _FrSky X4R-SB_ (S-BUS, düşük gecikme) veya _X4R_ (PPM-Sum, eski tip) alıcılarla kutudan çıktığı gibi kullanılabilir.
Ayrıca özel bir radyo verici modülü yuvasına ve özelleştirilebilir açık kaynaklı OpenTX Firmware'e sahiptir.

::: info
Bu uzaktan kontrol ünitesi, [FrSky](../peripherals/frsky_telemetry.md) veya [TBS Crossfire](../telemetry/crsf_telemetry.md) radyo modülleri ile kullanıldığında araç telemetrisini görüntüleyebilir.
:::

Diğer popüler verici/alıcı çiftleri:

- Örneğin FrSky verici/alıcı modüllerini kullanan Turnigy kumandaları.
- Futaba Vericileri ve uyumlu Futaba S-Bus alıcıları.
- Uzun menzilli ~900MHz, düşük gecikmeli: Uyumlu bir kumanda (örn. Taranis) ile "Team Black Sheep Crossfire" veya "Crossfire Micro" seti.
- Uzun Menzilli ~433MHz: Uyumlu bir kumanda (örn. Taranis) ile ImmersionRC EzUHF seti.

### PX4 Uyumlu Alıcılar {#compatible_receivers}

Verici/alıcı çiftlerinin uyumlu olmasının yanı sıra, alıcının PX4 ve uçuş kontrol donanımıyla da uyumlu olması gerekir.

_PX4_ ve _Pixhawk_ şunlarla doğrulanmıştır:

- PPM sum (toplam) alıcıları
- Aşağıdakilerden S.BUS ve S.BUS2 alıcıları:
  - Futaba
  - FrSky S.BUS ve PPM modelleri
  - Çıkış protokolü olarak SBUS kullanan TBS Crossfire
  - Herelink

- ([CRSF protokolü](../telemetry/crsf_telemetry.md)) ile TBS Crossfire
- ([CRSF protokolü](../telemetry/crsf_telemetry.md)) ile Express LRS

- (GHST protokolü) ile TBS Ghost
- Spektrum DSM
- Graupner HoTT

Desteklenen bir protokolü kullanan diğer satıcıların alıcılarının çalışması muhtemeldir ancak test edilmemiştir.

::: info
Tarihsel olarak, büyük ölçüde protokollerin ayrıntılı spesifikasyon eksikliği nedeniyle alıcı modelleri arasında farklılıklar ve uyumsuzluklar vardı.
Test ettiğimiz alıcıların hepsi artık uyumlu görünüyor, ancak diğerlerinin olmaması mümkündür.
:::

## Alıcıları Bağlama

Genel bir rehber olarak, alıcılar destekledikleri protokole uygun portu kullanarak uçuş kontrolcüsüne bağlanır:

- Spektrum/DSM alıcıları "DSM" girişine bağlanır.
  Pixhawk uçuş kontrolcüleri bunu çeşitli şekillerde etiketler: `SPKT/DSM`, `DSM`, `DSM/SBUS RC`, `DSM RC`, `DSM/SBUS/RSSI`.
- Graupner HoTT alıcıları: SUMD çıkışı bir **SPKT/DSM** girişine bağlanmalıdır (yukarıdaki gibi).
- PPM-Sum ve S.BUS alıcıları doğrudan **RC** toprak, güç ve sinyal pinlerine bağlanmalıdır.
  Bu genellikle şöyle etiketlenir: `RC IN`, `RCIN` veya `RC`, ancak bazı FC'lerde `PPM RC` veya `PPM` olarak etiketlenmiştir.
- Her kanal için ayrı bir kabloya sahip olan PPM alıcıları, RCIN kanalına [bunun gibi](https://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) bir PPM kodlayıcı (encoder) _aracılığıyla_ bağlanmalıdır (PPM-Sum alıcıları tüm kanallar için tek bir sinyal kablosu kullanır).
- [CRSF Telemetri](../telemetry/crsf_telemetry.md) kullanan TBS Crossfire/Express LRS Alıcıları boş bir UART üzerinden bağlanır.

Uçuş kontrolcüleri genellikle yaygın alıcı türlerini bağlamak için uygun kabloları içerir.

Belirli uçuş kontrolcülerine bağlantı talimatları, onların [hızlı başlangıç](../assembly/index.md) kılavuzlarında verilmiştir (örneğin [CUAV Pixhawk V6X Kablolama Hızlı Başlangıç: Radyo Kontrol](../assembly/quick_start_cuav_pixhawk_v6x.md#radio-control) veya [Holybro Pixhawk 6X Kablolama Hızlı Başlangıç: Radyo Kontrol](../assembly/quick_start_pixhawk6x.md#radio-control)).

:::tip
Ek bilgi için üreticinin uçuş kontrolcüsü kurulum kılavuzuna bakın.
:::

<a id="binding"></a>

## Verici/Alıcı Eşleştirme (Binding)

Bir radyo sistemini kalibre etmeden/kullanmadan önce, alıcı ve vericiyi birbirleriyle iletişim kuracak şekilde _eşleştirmeniz_ (bind etmeniz) gerekir.
Bir verici ve alıcı çiftini eşleştirme işlemi donanıma özgüdür (talimatlar için kılavuzunuza bakın).

Bir _Spektrum_ alıcı kullanıyorsanız, _QGroundControl_ kullanarak onu eşleştirme moduna (bind mode) alabilirsiniz: [Radyo Kurulumu > Spectrum Eşleştirme](../config/radio.md#spectrum-bind).

## Sinyal Kaybı Davranışını Ayarlama

RC alıcılarının sinyal kaybını belirtmek için farklı yolları vardır:

- Hiçbir şey göndermemek (PX4 tarafından otomatik olarak algılanır)
- Düşük bir gaz (throttle) değeri göndermek (PX4'ü [bunu algılayacak şekilde yapılandırabilirsiniz](../config/radio.md#rc-loss-detection)).
- Son alınan sinyali göndermek (PX4 bu durumu yönetemez!)

RC kaybolduğunda hiçbir şey yaymayan (tercih edilen) veya düşük bir gaz değeri gönderen bir alıcı seçin.
Bu davranış, alıcının donanım yapılandırmasını gerektirebilir (kılavuzu kontrol edin).

Daha fazla bilgi için bkz. [Radyo Kontrol Kurulumu > RC Kaybı Algılama](../config/radio.md#rc-loss-detection).

## İlgili Konular

- [Radyo Kontrol Kurulumu](../config/radio.md) - Radyonuzu PX4 ile yapılandırma.
- [Çok rotorlu](../flying/basic_flying_mc.md) veya [sabit kanatlı](../flying/basic_flying_fw.md) araçlarda Manuel Uçuş - Uzaktan kumanda ile nasıl uçulacağını öğrenin.
- [TBS Crossfire (CRSF) Telemetrisi](../telemetry/crsf_telemetry.md)
- [FrSky Telemetrisi](../peripherals/frsky_telemetry.md)