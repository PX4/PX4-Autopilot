# 진동 방지

이 절에서는 진동의 세기를 판단 방법과 진동 특성 개선 방법을 설명합니다.

## 개요

비행 콘트롤러에 장착된 가속 센서나 자이로 회전 센서는 진동에 매우 민감합니다.
큰 진동은 비행 효율 저하, 성능 감소, 비행 시간 단축, 기체 내구도 감소 등의 다양한 문제을 일으킵니다. 극단적인 경우에는 진동으로 인하여 센서가 오작동하거나 파손될 수 있으며, 이로 인하여 자세/위치 추정에 실패하여 기체가 멀리 날아가 버릴 수 있습니다.

잘 설계된 기체는 비행 콘트롤러 장착된 근방의 진동의 구조적 공명 진폭을 감쇠시킵니다.
몇몇 비행 콘트롤러는 반진동 폼을 사용하여 기체에 장착하여야 합니다. 민감한 장치들은 충분히 감당할 수 있는 수준까지 진동을 줄이기 위하여 추가적인 방법이 필요합니다.

## 진동 분석

[Log Analysis using Flight Review > Vibration](../log/flight_review.md#vibration) explains how to use logs to confirm whether vibration is a probable cause of flight problems.

## 기초적인 진동 해결 방법

간단한 몇 가지의 단계로 진동을 줄일 수 있습니다:

- 모든 장비(랜딩 기어, GPS 지지대 등)가 기체에 단단히 고정되어 있는 지 확인하십시오.
- 균형 잡힌 프로펠러를 사용하십시오.
- 고품질의 프로펠러, 모터, ESC와 기체 프레임을 사용하십시오.
  품질에서 큰 차이가 발생합니다.
- 비행 콘트롤러 장착시에 방진이 필요합니다.
  Many flight controllers come with _mounting foam_ that you can use for this purpose, while others have inbuilt vibration-isolation mechanisms.
- As a _last_ measure, adjust the [software filters](../config_mc/filter_tuning.md).
  그러나, 진동 원인을 근원적으로 제거하는 것이 소프트웨어 필터링보다 좋은 방법입니다.

## 참고

유용한 참고사항들입니다.

- [An Introduction to Shock & Vibration Response Spectra, Tom Irvine](http://www.vibrationdata.com/tutorials2/srs_intr.pdf) (free paper)
- Structural Dynamics and Vibration in Practice - An Engineering Handbook, Douglas Thorby (preview).
