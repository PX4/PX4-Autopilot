<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div style="float:right; padding:10px; margin-right:20px;"><a href="https://px4.io/"><img src="../assets/site/logo_pro_small.png" title="PX4 Logo" width="180px" /></a></div>

# PX4 Otopilot Kullanıcı Rehberi

[![Releases](https://img.shields.io/badge/release-main-blue.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](https://discuss.px4.io//) [![Discord](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

PX4, _Profesyonel Otopilot_ çözümüdür.
Endüstri ve akademiden dünya standartlarında geliştiriciler tarafından geliştirilen ve dünya çapında aktif bir topluluk tarafından desteklenen PX4; yarış ve kargo dronlarından kara araçlarına ve denizaltılara kadar her türlü araca güç verir.

:::tip
Bu rehber, PX4 tabanlı bir aracı monte etmek, yapılandırmak ve güvenli bir şekilde uçurmak için ihtiyacınız olan her şeyi içerir.
Katkıda bulunmak ister misiniz? [Geliştirme](development/development.md) bölümüne göz atın.
:::

<div v-if="site.title == 'PX4 Guide (main)'">

:::warning
Bu rehber, PX4'ün _geliştirme_ sürümü (`main` dalı) içindir.
Mevcut _kararlı_ (stable) sürümü bulmak için **Sürüm** seçiciyi kullanın.

Kararlı sürümden bu yana yapılan belgelenmiş değişiklikler, gelişen [sürüm notlarında](releases/main.md) yer almaktadır.
:::

</div>

## Nasıl Başlarım?

[Temel Kavramlar](getting_started/px4_basic_concepts.md) tüm kullanıcılar tarafından okunmalıdır!
Bu bölüm; uçuş yığını (uçuş modları ve güvenlik özellikleri) ve desteklenen donanımlar (uçuş kontrolcüsü, araç tipleri, telemetri sistemleri, RC kontrol sistemleri) tarafından sağlanan özellikler de dahil olmak üzere PX4'e genel bir bakış sağlar.

Ne başarmak istediğinize bağlı olarak, aşağıdaki ipuçları bu rehberde gezinmenize yardımcı olacaktır:

### PX4 ile çalışan bir araç istiyorum

[Çok Rotorlu (Multicopter)](frames_multicopter/index.md), [VTOL](frames_vtol/index.md) ve [Uçak (Sabit Kanat)](frames_plane/index.md) bölümlerinde aşağıdakiler gibi konular bulacaksınız (bu linkler çok rotorlular içindir):

- [Tam Araçlar](complete_vehicles_mc/index.md) "Uçuşa Hazır" (RTF) önceden üretilmiş araçları listeler.
- [Kitler](frames_multicopter/kits.md) önceden seçilmiş parçalardan oluşan ve sizin birleştirmeniz gereken dronları listeler.
- [Kendin Yap (DIY) Montajlar](frames_multicopter/diy_builds.md) tek tek temin edilen parçalar kullanılarak inşa edilmiş bazı dron örneklerini gösterir.

Hem kitler hem de tam araçlar, genellikle pil ve RC (Kumanda) Sistemi hariç ihtiyacınız olan her şeyi içerir.
Kitlerin yapımı genellikle zor değildir, dronların nasıl bir araya geldiğine dair iyi bir giriş sağlar ve nispeten ucuzdur.
[Çok Rotorlu Montajı](assembly/assembly_mc.md) gibi genel montaj talimatları sağlıyoruz ve çoğu kitin kendi özel talimatları da vardır.

Eğer kitler ve hazır dronlar size tam olarak uymuyorsa, sıfırdan bir araç yapabilirsiniz ancak bu daha fazla bilgi gerektirir.
[Gövde Montajları](airframes/index.md), neyin mümkün olduğu konusunda size fikir vermek için desteklenen gövde başlangıç noktalarını listeler.

PX4'ü destekleyen bir araca sahip olduğunuzda, onu yapılandırmanız ve sensörleri kalibre etmeniz gerekecektir.
Her araç tipinin, [Çok Rotorlu Yapılandırma/Ayarlama](config_mc/index.md) gibi ana adımları açıklayan kendi yapılandırma bölümü vardır.

### Faydalı yük/kamera eklemek istiyorum

[Faydalı Yükler](payloads/index.md) bölümü, bir kameranın nasıl ekleneceğini ve PX4'ün paket teslimatı yapabilmeniz için nasıl yapılandırılacağını açıklar.

### Desteklenen bir aracı modifiye ediyorum

[Donanım Seçimi ve Kurulumu](hardware/drone_parts.md) bölümü, PX4 ile kullanabileceğiniz donanımlar ve yapılandırmaları hakkında hem üst düzey hem de ürüne özgü bilgiler sağlar.
Bir dronu modifiye etmek ve yeni bileşenler eklemek istiyorsanız bakmanız gereken ilk yer burasıdır.

### Uçmak istiyorum

Uçmadan önce, aracınızın güvenlik özelliklerini ve tüm gövde tiplerinin ortak davranışlarını anlamak için [Operasyonlar](config/operations.md) bölümünü okumalısınız.
Bunu yaptıktan sonra uçmaya hazırsınız.

Her araç tipi için temel uçuş talimatları, [Temel Uçuş (Çok Rotorlu)](flying/basic_flying_mc.md) gibi ilgili bölümlerde verilmiştir.

### PX4'ü yeni bir Uçuş Kontrolcüsünde çalıştırmak ve platformu genişletmek istiyorum

[Geliştirme](development/development.md) bölümü; yeni gövdeleri ve araç tiplerini nasıl destekleyeceğinizi, uçuş algoritmalarını nasıl değiştireceğinizi, yeni modlar eklemeyi, yeni donanımları entegre etmeyi, uçuş kontrolcüsünün dışından PX4 ile iletişim kurmayı ve PX4'e nasıl katkıda bulunacağınızı açıklar.

## Yardım Alma

[Destek](contribute/support.md) sayfası, çekirdek geliştirme ekibinden ve daha geniş topluluktan nasıl yardım alacağınızı açıklar.

Diğer şeylerin yanı sıra şunları kapsar:

- [Yardım alabileceğiniz forumlar](contribute/support.md#forums-and-chat)
- [Sorunları teşhis etme](contribute/support.md#diagnosing-problems)
- [Hatalar nasıl bildirilir](contribute/support.md#issue-bug-reporting)
- [Haftalık geliştirici görüşmesi](contribute/support.md#weekly-dev-call)

## Hata ve Sorun Bildirimi

PX4'ü kullanırken herhangi bir sorun yaşarsanız, önce bunları [destek forumlarında](contribute/support.md#forums-and-chat) paylaşın (çünkü bunlar araç yapılandırmasından kaynaklanıyor olabilir).

Geliştirme ekibi tarafından yönlendirilirse, kod sorunları [GitHub üzerinden burada](https://github.com/PX4/PX4-Autopilot/issues) açılabilir.
Mümkün olduğunda [uçuş loglarını](getting_started/flight_reporting.md) ve sorun şablonunda istenen diğer bilgileri sağlayın.

## Katkıda Bulunma

Koda ve dokümantasyona nasıl katkıda bulunulacağına dair bilgiler [Katkıda Bulunma](contribute/index.md) bölümünde bulunabilir:

- [Kod](contribute/index.md)
- [Dokümantasyon](contribute/docs.md)
- [Çeviri](contribute/translation.md)

## Çeviriler

Bu rehberin çeşitli [çevirileri](contribute/translation.md) mevcuttur.
Bunlara Diller menüsünden (sağ üst) erişebilirsiniz:

![Language Selector](../assets/vuepress/language_selector.png)

## Lisans

PX4 kodu, izin veren [BSD 3-maddeli lisans](https://opensource.org/license/BSD-3-Clause) şartları altında kullanmak ve değiştirmek için ücretsizdir.
Bu dokümantasyon [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) altında lisanslanmıştır.
Daha fazla bilgi için bkz: [Lisanslar](contribute/licenses.md).

## Takvim ve Etkinlikler

_Dronecode Takvimi_, platform kullanıcıları ve geliştiricileri için önemli topluluk etkinliklerini gösterir.
Takvimi kendi saat diliminizde görüntülemek (ve kendi takviminize eklemek) için aşağıdaki bağlantıları seçin:

- [İsviçre – Zürih](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
- [Pasifik Saati – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
- [Avustralya – Melbourne/Sidney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

:::tip
Takvim varsayılan saat dilimi Orta Avrupa Zaman Dilimi'dir (CET).
:::

<iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no"></iframe>

### İkonlar

Bu kütüphanede kullanılan aşağıdaki ikonlar ayrı olarak lisanslanmıştır (aşağıda gösterildiği gibi):

<img src="../assets/site/position_fixed.svg" title="Konum sabitlemesi gerekli (ör. GPS)" width="30px" /> _placeholder_ ikonu: <a href="https://www.flaticon.com/authors/smashicons" title="Smashicons">Smashicons</a> tarafından <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a> üzerinden yapılmıştır ve <a href="https://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a> ile lisanslanmıştır.

<img src="../assets/site/automatic_mode.svg" title="Otomatik mod" width="30px" /> _camera-automatic-mode_ ikonu: <a href="https://www.freepik.com" title="Freepik">Freepik</a> tarafından <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a> üzerinden yapılmıştır ve <a href="https://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a> ile lisanslanmıştır.

## Yönetişim

PX4 uçuş yığını, [Dronecode Projesi](https://dronecode.org/)'nin yönetimi altında barındırılmaktadır.

<a href="https://dronecode.org/" style="padding:20px" ><img src="../assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="../assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>

Doc build time: {{ $buildTime }}