# Uçuş Raporlama

PX4, performans sorunlarını analiz etmek için kullanılabilecek ayrıntılı hava aracı durum ve sensör verilerini kaydeder (loglar).
Bu konu, logları nasıl indirip analiz edebileceğinizi ve inceleme için geliştirme ekibiyle nasıl paylaşabileceğinizi açıklar.

:::tip
Uçuş kayıtlarını (loglarını) tutmak bazı yargı bölgelerinde yasal bir zorunluluktur.
:::

## Logları Uçuş Kontrolcüsünden İndirme

Loglar [QGroundControl](https://qgroundcontrol.com/) kullanılarak indirilebilir: **[Analyze View > Log Download](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/log_download.html)**.

![Flight Log Download](../../assets/qgc/analyze/log_download.jpg)

::: tip
Şifrelenmiş loglar QGroundControl ile indirilemez veya herkese açık Flight Review hizmetine yüklenemez.
Şifrelenmiş logları indirmenin ve ayıklamanın en kolay yolu [Log Şifreleme Araçlarını](../dev_log/log_encryption.md) kullanmaktır.
Ayrıca, yükleme sırasında özel anahtarınızı kullanarak logların şifresini otomatik olarak çözen [özel bir Flight Review sunucusu](../dev_log/log_encryption.md#flight-review-encrypted-logs) da barındırabilirsiniz.
:::

## Logları Analiz Etme

Log dosyasını çevrimiçi [Flight Review](https://logs.px4.io/) aracına yükleyin.
Yüklemeden sonra, logun analiz sayfasına giden bir bağlantı size e-posta ile gönderilecektir.

[Flight Review kullanarak Log Analizi](../log/flight_review.md), grafikleri nasıl yorumlayacağınızı açıklar ve yaygın sorunların nedenlerini doğrulamanıza/reddetmenize yardımcı olabilir: aşırı titreşim, kötü PID ayarı, doymuş kontrolcüler (saturated), dengesiz araçlar, GPS gürültüsü vb.

::: info
PX4 Loglarını görselleştirmek ve analiz etmek için başka harika araçlar da vardır.
Daha fazla bilgi için bkz: [Uçuş Analizi](../dev_log/flight_log_analysis.md).
:::

:::tip
Araca (sadece bir telemetri bağlantısı değil) sürekli ve yüksek hızlı bir MAVLink bağlantınız varsa, logları doğrudan _Flight Review_'a otomatik olarak yüklemek için _QGroundControl_'u kullanabilirsiniz.
Daha fazla bilgi için bkz. [Settings > MAVLink Settings > MAVLink 2 Logging (PX4 only)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/mavlink.html#logging).
:::

## Log Dosyalarını PX4 Geliştiricileri Tarafından İncelenmesi İçin Paylaşma

[Flight Review](https://logs.px4.io/) log dosyası bağlantısı, [destek forumlarında](../contribute/support.md#forums-and-chat) veya bir [GitHub konusunda (issue)](../index.md#reporting-bugs-issues) tartışmak üzere paylaşılabilir.

## Log Yapılandırması

Loglama sistemi, varsayılan olarak [Flight Review](https://logs.px4.io/) ile kullanım için mantıklı loglar toplayacak şekilde yapılandırılmıştır.

Loglama, [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parametreleri kullanılarak veya SD karttaki bir dosya ile daha fazla yapılandırılabilir.
Yapılandırma ile ilgili ayrıntılar [loglama yapılandırması dokümantasyonunda](../dev_log/logging.md#configuration) bulunabilir.

## Önemli Bağlantılar

- [Flight Review](https://logs.px4.io/)
- [Flight Review kullanarak Log Analizi](../log/flight_review.md)
- [Uçuş Log Analizi](../dev_log/flight_log_analysis.md)