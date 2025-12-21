# LED Anlamları (Pixhawk Serisi)

[Pixhawk serisi uçuş kontrolcüleri](../flight_controller/pixhawk_series.md), aracın mevcut durumunu göstermek için LED'leri kullanır.
- [UI LED (Kullanıcı Arayüzü LED'i)](#ui_led), *uçuşa hazır olma* ile ilgili kullanıcıya yönelik durum bilgilerini sağlar.
- [Durum LED'leri](#status_led), PX4IO ve FMU SoC (Çip Üzerinde Sistem) için durum bilgisi sağlar.
  Güç, önyükleyici (bootloader) modu, aktivite ve hataları gösterirler.

<a id="ui_led"></a>

## UI LED

RGB *UI LED*, aracın mevcut *uçuşa hazır olma* durumunu gösterir. 
Bu genellikle, uçuş kontrol kartına monte edilmiş veya edilmemiş olabilen (örn. FMUv4'te kart üzerinde yoktur ve genellikle GPS üzerine monte edilmiş bir LED kullanır) süper parlak bir I2C çevresel birimidir.

Aşağıdaki görsel, LED ve araç durumu arasındaki ilişkiyi göstermektedir.

:::warning
GPS kilidine (Yeşil LED) sahip olup da aracı "Arm" edememeniz (motorları başlatamamanız) mümkündür, çünkü PX4 henüz [uçuş öncesi kontrolleri](../flying/pre_flight_checks.md) geçmemiştir. **Kalkış için geçerli bir küresel konum tahmini gereklidir!**
:::

:::tip
Bir hata durumunda (yanıp sönen kırmızı) veya araç GPS kilidine ulaşamazsa (maviden yeşile dönmezse), kalibrasyon durumu ve [Uçuş Öncesi Kontroller (Dahili)](../flying/pre_flight_checks.md) tarafından bildirilen hata mesajları dahil olmak üzere daha ayrıntılı durum bilgisi için *QGroundControl*'ü kontrol edin. 
Ayrıca GPS modülünün düzgün takılıp takılmadığını, Pixhawk'ın GPS'inizi düzgün okuyup okumadığını ve GPS'in düzgün bir GPS konumu gönderip göndermediğini kontrol edin.
:::

![LED meanings](../../assets/flight_controller/pixhawk_led_meanings.gif)


* **[Sabit Mavi] Armed, GPS Kilidi Yok:** Aracın arm edildiğini (motorların etkinleştirildiğini) ve bir GPS ünitesinden konum kilidi olmadığını gösterir.
Araç arm edildiğinde, PX4 motorların kontrolünü açarak dronunuzu uçurmanıza izin verir.
Her zaman olduğu gibi, arm ederken dikkatli olun, çünkü büyük pervaneler yüksek devirlerde tehlikeli olabilir.
Araç bu modda güdümlü (guided) görevleri gerçekleştiremez.

* **[Yanıp Sönen Mavi] Disarmed, GPS Kilidi Yok:** Yukarıdakine benzer, ancak aracınız disarmed (motorlar devre dışı) durumdadır.
Bu, motorları kontrol edemeyeceğiniz, ancak diğer tüm alt sistemlerin çalıştığı anlamına gelir.

* **[Sabit Yeşil] Armed, GPS Kilidi Var:** Aracın arm edildiğini ve bir GPS ünitesinden geçerli bir konum kilidine sahip olduğunu gösterir.
Araç arm edildiğinde, PX4 motorların kontrolünü açarak dronunuzu uçurmanıza izin verir.
Her zaman olduğu gibi, arm ederken dikkatli olun, çünkü büyük pervaneler yüksek devirlerde tehlikeli olabilir.
Bu modda araç güdümlü görevleri gerçekleştirebilir.

* **[Yanıp Sönen Yeşil] Disarmed, GPS Kilidi Var:** Yukarıdakine benzer, ancak aracınız disarmed durumdadır.
  Bu, motorları kontrol edemeyeceğiniz, ancak GPS konum kilidi dahil diğer tüm alt sistemlerin çalıştığı anlamına gelir.

* **[Sabit Mor] Failsafe (Arıza Koruma) Modu:** Bu mod, araç uçuş sırasında manuel kontrol kaybı, kritik derecede düşük pil veya dahili bir hata gibi bir sorunla karşılaştığında etkinleşir.
Failsafe modu sırasında araç kalkış konumuna dönmeye çalışacak veya olduğu yere inecektir.

* **[Sabit Kehribar (Sarı)] Düşük Pil Uyarısı:** Aracınızın pilinin tehlikeli derecede azaldığını gösterir.
Belirli bir noktadan sonra araç failsafe moduna geçer. Ancak, bu mod bu uçuşu sonlandırma zamanının geldiğine dair bir ikaz sinyali olmalıdır.

* **[Yanıp Sönen Kırmızı] Hata / Kurulum Gerekli:** Otopilotunuzun uçmadan önce yapılandırılması veya kalibre edilmesi gerektiğini gösterir.
Sorunun ne olduğunu doğrulamak için otopilotunuzu bir Yer Kontrol İstasyonuna bağlayın.
Kurulum işlemini tamamladıysanız ve otopilot hala kırmızı ve yanıp sönüyor görünüyorsa, başka bir hata olabilir.


<a id="status_led"></a>

## Durum LED'i (Status LED)

Üç *Durum LED'i*, FMU SoC için durum sağlar ve üç tane daha PX4IO (eğer varsa) için durum sağlar. 
Güç, önyükleyici (bootloader) modu, aktivite ve hataları gösterirler.

![Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_status_leds.jpg)

Güç açıldığında, FMU ve PX4IO işlemcileri önce önyükleyiciyi (Bootloader - BL) ve ardından uygulamayı (APP) çalıştırır.
Aşağıdaki tablo, Bootloader ve ardından APP'nin durumu göstermek için LED'leri nasıl kullandığını gösterir.

Renk | Etiket | Bootloader kullanımı | APP kullanımı 
--- | --- | --- | ---
Mavi | ACT (Aktivite) | Önyükleyici veri alırken titrer | ARM durumunun göstergesi
Kırmızı/Sarı | B/E (Önyükleyicide / Hata) | Önyükleyicideyken titrer | Bir HATA (ERROR) göstergesi
Yeşil |PWR (Güç) | Önyükleyici tarafından kullanılmaz | ARM durumunun göstergesi

::: info
Yukarıda gösterilen LED etiketleri yaygın olarak kullanılır, ancak bazı kartlarda farklılık gösterebilir.
:::

LED'lerin nasıl yorumlanacağına dair daha ayrıntılı bilgi aşağıda verilmiştir (burada "x", "herhangi bir durum" anlamına gelir).

Kırmızı/Sarı | Mavi |  Yeşil | Anlamı
--- | --- | --- | ---
10Hz | x | x | Aşırı Yük CPU yükü > %80 veya RAM kullanımı > %98
KAPALI | x | x | Aşırı Yük CPU yükü <= %80 veya RAM kullanımı <= %98
NA | KAPALI | 4 Hz| actuator_armed->armed && failsafe (Armed ve Failsafe'de)
NA | AÇIK | 4 Hz | actuator_armed->armed && !failsafe (Armed ve Failsafe değil)
NA | KAPALI |1 Hz | !actuator_armed-> armed && actuator_armed->ready_to_arm (Disarmed ve Arm olmaya hazır)
NA | KAPALI |10 Hz | !actuator_armed->armed  && !actuator_armed->ready_to_arm (Disarmed ve Arm olmaya hazır değil)