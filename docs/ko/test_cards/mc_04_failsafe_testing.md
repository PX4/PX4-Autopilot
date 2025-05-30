# 시험 MC_04 - 안전 장치 시험

## Objective

Test RC loss, data link loss, and low battery failsafes.

## Preflight

- Verify RC Loss action is Return to Land
- Verify Data Link Loss action is Return to Land and the timeout is 10 seconds
- Verify Battery failsafe
    - Action is Return to Land
    - Battery Warn Level is 25%
    - Battery Failsafe Level is 20%
    - Battery Emergency Level is 15%

## Flight Tests

❏ 원격 조종기 연결 유실

&nbsp;&nbsp;&nbsp;&nbsp;❏ Take off in Altitude mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Move at least 20 meters away home position

&nbsp;&nbsp;&nbsp;&nbsp;❏Turn off RC and check the vehicle returns to home position, wait for the descent and turn on the RC and take over.

❏ Datalink Loss

&nbsp;&nbsp;&nbsp;&nbsp;❏ Disconnect telemetry, vehicle should return to home position after 10 seconds, wait for the descent and reconnect the telemetry radio

❏ Battery Failsafe

&nbsp;&nbsp;&nbsp;&nbsp;❏ Confirm the warning message is received in QGC

&nbsp;&nbsp;&nbsp;&nbsp;❏ Confirm the vehicle returns to land on failsafe level

&nbsp;&nbsp;&nbsp;&nbsp;❏ Confirm the vehicle lands on emergency land level
