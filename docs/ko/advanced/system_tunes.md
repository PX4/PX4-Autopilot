# 시스템 알림음

PX4 defines a number of [standard tones/tunes](../getting_started/tunes.md) that are used to provide audio notification for important system states and problems (e.g. system startup, arming success, battery warnings, etc.)

Tunes are specified using strings (in [ANSI Music notation](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt)) and played by code using the [tunes](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/tunes) library.
The tunes library also contains the list of default system tunes - see [lib/tunes/tune_definition.desc](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc).

PX4에는 기본 알림음 또는 사용자 정의 알림음을 재생(테스트)하는 모듈이 있습니다.

자체 알림음을 만들고 시스템 알림음을 추가/교체하는 방법을 설명합니다.

## 알림음 만들기

Tune strings are defined using [ANSI Music notation](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt).

:::tip
More information about the format can be found in [QBasic PLAY statement](https://en.wikibooks.org/wiki/QBasic/Appendix#PLAY) (Wikibooks) and has been reproduced in [tune_definition.desc](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc).
:::

새로운 곡을 만드는 가장 쉬운 방법은 음악 편집기를 사용하는 것입니다.
이를 통해 음악을 편집하고 컴퓨터에서 재생한 다음 PX4에서 재생할 수 있는 형식으로 내보낼 수 있습니다.

ANSI 음악은 ANSI BBS 시스템 시대에 인기가 있었고, 최고의 편집 도구는 DOS 유틸리티입니다.
On Windows, one option is to use _Melody Master_ within _Dosbox_.

소프트웨어 사용 절차는 다음과 같습니다.

1. Download [DosBox](http://www.dosbox.com/) and install the app

2. Download [Melody Master](ftp://archives.thebbs.org/ansi_utilities/melody21.zip) and unzip into a new directory

3. Open the _Dosbox_ console

4. 멜로디 마스터 디렉터리를 아래와 같이 도스박스에서 마운트하십시오.

   ```sh
   mount c C:\<path_to_directory\Melody21
   ```

5. Start _Melody Master_ with the following commands

   ```sh
   c:
   start
   ```

6. You will then have the option to click through a few screens, then press **1** to display _Melody Master_:
   ![Melody Master 2.1](../../assets/tunes/tunes_melody_master_2_1.jpg)

   화면의 절반 하단부에서 도구 사용에 필요한 키보드 단축키를 안내해줍니다(악보를 움직이고 음표 길이를 선택할 수 있는 등의 작업 가능).

7. 음악을 저장할 준비가 끝나면:
   - Press **F2** to give the tune a name and save it in the _/Music_ sub folder of your Melody Master installation.
   - Press **F7**, the scroll down the list of output formats on the right to get to ANSI.
      The file will be exported to the _root_ of the Melody Master directory (with the same name and a file-type specific extension).

8. 파일을 여십시오.
   출력 내용은 다음과 같습니다:

   ![ANSI Output from file](../../assets/tunes/tune_musicmaker_ansi_output.png)

9. The string that can be played in PX4 is the bit between `MNT` and `P64`: `150L1O3DL16CL32<B>C<AEL16A`

## 알림음 시험

When you're ready to try it out a new tune on PX4, use the [tune_control](../modules/modules_system.md#tune-control) library.
For example, to test the tune we "created" above you would enter the following command on a console or shell (e.g. the [MAVLink Shell](../debug/mavlink_shell.md)):

```sh
tune_control play -m "150L1O3DL16CL32<B>C<AEL16A"
```

:::info
Out of the box, the `tune_control` is only present on real hardware (not the simulator).
:::

## 기존 알림음 변경

Tunes are defined within [tune_definition.desc](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc).

If you just need to replace an existing tune, then you can replace the file in your own fork, and update the tune strings defined in `PX4_DEFINE_TUNE`.

## 새 알림음 추가

곧 추가 예정.

<!--

1. Assumption is that you need to define a new `PX4_DEFINE_TUNE` with its own number in the file.
2. Need to look at how tunes are played. Problem for another day.

-->
