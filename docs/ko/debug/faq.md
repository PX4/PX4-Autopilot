# 자주 묻는 질문

## 빌드 오류

### 플래시 오버플로우

보드에 로딩 가능한 코드의 양은 보드에 있는 플래시 메모리의 양에 따라 제한됩니다.
모듈 또는 코드를 추가시, 플래시 메모리를 초과할 가능성이 있습니다.
This will result in a "flash overflow". 업스트림 버전은 항상 빌드되지만, 개발자가 추가하는 항목에 따라 로컬에서 오버플로될 수 있습니다.

```sh
region `flash' overflowed by 12456 bytes
```

이를 해결하려면, 최신 하드웨어를 사용하거나 비필수 모듈을 빌드에서 제거하여야 합니다.
The configuration is stored in **/PX4-Autopilot/boards/px4** (e.g. [PX4-Autopilot/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)).
모듈을 제거하려면, 다음과 같이 주석 처리하십시오.

```cmake
#tune_control
```

#### 대용량 메모리 소비 모듈 식별

아래 명령은 가장 큰 정적 할당을 출력합니다.

```sh
arm-none-eabi-nm --size-sort --print-size --radix=dec build/px4_fmu-v5_default/px4_fmu-v5_default.elf | grep " [bBdD] "
```

## USB 오류

### 업로드가 성공하지 못하였습니다

Ubuntu에서 모뎀 관리자를 제거합니다.

```sh
sudo apt-get remove modemmanager
```
