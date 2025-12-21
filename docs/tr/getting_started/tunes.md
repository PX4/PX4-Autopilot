# Melodi Anlamları (Pixhawk Serisi)

[Pixhawk serisi uçuş kontrolcüleri](../flight_controller/pixhawk_series.md), araç durumunu ve olayları (örn. arm etme başarısı ve başarısızlığı, düşük pil uyarıları) belirtmek için bir [buzzer](../getting_started/px4_basic_concepts.md#buzzer)'dan gelen sesli tonları/melodileri ve bir [LED](../getting_started/led_meanings.md)'den gelen renkleri/dizileri kullanır.

Standart ses seti aşağıda listelenmiştir.

::: info
**Geliştiriciler:** Melodiler [/lib/tunes/tune_definition.desc](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc) içinde tanımlanmıştır ve [tune-control](../modules/modules_system.md#tune-control) modülü kullanılarak test edilebilir.
`TUNE_ID_name` dizesini (örn. `TUNE_ID_PARACHUTE_RELEASE`) kullanarak melodi kullanımını arayabilirsiniz.
:::


## Önyükleme/Başlangıç (Boot/Startup)

Bu melodiler önyükleme (boot) sırası sırasında çalınır.
#### Başlangıç Tonu

<audio controls><source src="../../assets/tunes/1_startup_tone.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- microSD kart başarıyla bağlandı (mount edildi) (önyükleme sırasında).

#### Hata Melodisi

<audio controls><source src="../../assets/tunes/2_error_tune.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Sert bir hata (Hard fault) sistemin yeniden başlatılmasına neden oldu.
- Sistem PX4IO kullanacak şekilde ayarlandı ancak IO mevcut değil.
- UAVCAN etkin ancak sürücü başlatılamıyor.
- SITL/HITL etkin ancak *pwm_out_sim* sürücüsü başlatılamıyor.
- FMU başlangıcı başarısız oldu.


#### Dosya Sistemi Oluşturma

<audio controls><source src="../../assets/tunes/16_make_fs.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- microSD kart biçimlendiriliyor. 
- Bağlama (mount) başarısız oldu (biçimlendirme başarılı olursa önyükleme sırası tekrar bağlamayı deneyecektir).
- microSD kart yok.


#### Biçimlendirme Başarısız

<audio controls><source src="../../assets/tunes/17_format_failed.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- microSD kartı biçimlendirme başarısız oldu (önceki kart bağlama girişiminin ardından).


#### PX4IO Programlama

<audio controls><source src="../../assets/tunes/18_program_px4io.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- PX4IO programlanmaya başlanıyor.

#### PX4IO Programlama Başarılı

<audio controls><source src="../../assets/tunes/19_program_px4io_success.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- PX4IO programlama başarılı oldu.

#### PX4IO Programlama Başarısız

<audio controls><source src="../../assets/tunes/20_program_px4io_fail.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- PX4IO programlama başarısız oldu.
- PX4IO başlatılamadı.
- AUX Mikser bulunamadı.


## Operasyonel

Bu tonlar/melodiler normal çalışma sırasında çıkarılır.

<a id="error_tune_operational"></a>
#### Hata Melodisi

<audio controls><source src="../../assets/tunes/2_error_tune.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- RC (Kumanda) Kaybı

#### Olumlu Bildirim Tonu

<audio controls><source src="../../assets/tunes/3_notify_positive_tone.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Kalibrasyon başarılı.
- Başarılı mod değişikliği.
- Komut kabul edildi (örn. MAVLink komut protokolünden).
- Güvenlik anahtarı kapalı (araç arm edilebilir).

#### Nötr Bildirim Tonu

<audio controls><source src="../../assets/tunes/4_notify_neutral_tone.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Görev geçerli ve uyarısı yok.
- Hava hızı kalibrasyonu: daha fazla hava basıncı sağlayın veya kalibrasyon tamamlandı.
- Güvenlik anahtarı açıldı/disarmed (araca yaklaşmak güvenli).

#### Olumsuz Bildirim Tonu

<audio controls><source src="../../assets/tunes/5_notify_negative_tone.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Kalibrasyon başarısız.
- Kalibrasyon zaten tamamlandı.
- Görev geçersiz.
- Komut reddedildi, başarısız oldu, geçici olarak reddedildi (örn. MAVLink komut protokolünden).
- Arm/disarm geçişi reddedildi (örn. uçuş öncesi kontroller başarısız, güvenlik devre dışı değil, sistem manuel modda değil).
- Mod geçişi reddedildi.

#### Arm Uyarısı

<audio controls><source src="../../assets/tunes/6_arming_warning.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Araç şu an armed (kurulu/etkin).

#### Arm Başarısız Melodisi

<audio controls><source src="../../assets/tunes/10_arming_failure_tune.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Arm işlemi başarısız oldu

#### Pil Uyarısı Yavaş

<audio controls><source src="../../assets/tunes/7_battery_warning_slow.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Düşük pil uyarısı ([failsafe](../config/safety.md#battery-level-failsafe)).

#### Pil Uyarısı Hızlı

<audio controls><source src="../../assets/tunes/8_battery_warning_fast.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Kritik düşük pil uyarısı ([failsafe](../config/safety.md#battery-level-failsafe)).


#### GPS Uyarısı Yavaş

<audio controls><source src="../../assets/tunes/9_gps_warning_slow.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
#### Paraşüt Serbest Bırakma

<audio controls><source src="../../assets/tunes/11_parachute_release.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Paraşüt serbest bırakma tetiklendi.


#### Tek Bip

<audio controls><source src="../../assets/tunes/14_single_beep.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Manyetometre/Pusula kalibrasyonu: Kullanıcıya aracı döndürmeye başlamasını bildirir.

#### Ev Konumu Ayarlandı Melodisi

<audio controls><source src="../../assets/tunes/15_home_set_tune.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>
- Ev konumu (Home position) başlatıldı (yalnızca ilk seferde).

#### Güç Kapatma Melodisi

<audio controls><source src="../../assets/tunes/power_off_tune.mp3" type="audio/mpeg">Tarayıcınız ses elementini desteklemiyor.</audio>

- Araç gücü kapatılıyor.