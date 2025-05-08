# PX4 SD 카드 디렉토리 구조

PX4에서는 SD 카드에 구성 파일, 비행 로그, 임무 정보 등을 저장합니다.

:::tip
The SD card should be FAT32 formatted for use with PX4 (this is the default for SD cards).
다른 파일 시스템의 카드는 다시 포맷하여야 합니다.
:::

디렉토리 구조는 아래와 같습니다.

| 디렉토리/파일                 | 설명                                                                                       |
| ----------------------- | ---------------------------------------------------------------------------------------- |
| `/etc/`                 | Extra config. See [System Startup > Replacing the System Startup][replace system start]. |
| `/log/`                 | Full [flight logs](../dev_log/logging.md)                                                |
| `/mission_log/`         | 일부 비행 기록                                                                                 |
| `/fw/`                  | [DroneCAN](../dronecan/index.md) firmware                                                |
| `/uavcan.db/`           | DroneCAN DNA server DB + logs                                                            |
| `/params`               | 매개변수들 (FRAM/FLASH에 없을 경우)                                             |
| `/dataman`              | 임무 저장 파일                                                                                 |
| `/fault_<datetime>.txt` | 하드폴트 파일                                                                                  |
| `/bootlog.txt`          | 부팅 로그 파일                                                                                 |

[replace system start]: ../concept/system_startup.md#replacing-the-system-startup
