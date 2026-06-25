# カタパルト離陸自動化 — 実装ドキュメント

PX4 v1.16.1 / Pixhawk 6C Mini（fmu-v6c）向けに、筒射出（カタパルト）型固定翼機の
離陸を自動化する機能の実装記録。

## 1. 目的

筒からの射出後、以下を自動で行う：

1. **尾翼ロック解除サーボの解放**（MAIN5 / MAIN6）
2. **推進モーターの始動**（MAIN3）— 尾翼が解放されるまでは始動を抑止

射出（加速度）は PX4 標準の launch detector で検知し、そこから所定の遅延を経て
尾翼を解放し、モーターを通常スロットルに解禁する。

---

## 2. 2つのアプローチとブランチ構成

本リポジトリは git 管理（ベース: タグ `v1.16.1` / コミット `94cb201`）。
2つの実装方式をそれぞれ別ブランチで保持している。

| ブランチ | 方式 | 状態 |
|---|---|---|
| `catapult/approach-b-poc` | **Approach B**: 独立モジュール `catapult_launch` | PoC・SITL検証済み（保存用） |
| `catapult/approach-a` | **Approach A**: 既存 Takeoff 制御を拡張（本命） | 実装・SITL検証済み |

> **どちらを使うか**: 本番は **Approach A**。Approach B は設計検討用の PoC として残してある。
> 両者は同じ `CAT_*` パラメータ名を使うため、**同時にはビルドできない**
> （Approach A ブランチでは Approach B モジュールをビルドから除外している）。

---

## 3. Approach A（本命）— 既存 Takeoff への統合

独立モジュールを増やさず、既存の固定翼 Takeoff 制御
`FixedwingPositionControl::control_auto_takeoff()` の
**カタパルト/ハンドランチ・パス**（滑走離陸でない側）に処理を埋め込む方式。

### 3.1 動作シーケンス

`CAT_EN=1` かつ Takeoff 系 nav state のとき：

```
離陸パス進入
  └─ 尾翼ロックPWM出力（CAT_TAIL5_LCK / CAT_TAIL6_LCK）         … 一度だけ
        │
launch 検知（標準 launch detector が加速度しきい値を検知 = T0）
  └─ T0 を記録
        │
T0 + CAT_TAIL_DLY 経過
  └─ 尾翼解放PWM出力（CAT_TAIL5_REL / CAT_TAIL6_REL）           … 一度だけ
        │
尾翼解放まで：モーターは FW_THR_IDLE に固定（CAT_MOT_REQ_TAIL=1 のとき）
  └─ 解放後：通常の離陸スロットル（TECS）へ解禁
```

`CAT_EN=0` のときは上記を一切行わず、PX4 標準の launch/takeoff 挙動と完全に同一
（回帰テストで確認済み）。

### 3.2 アクチュエータ出力経路

尾翼サーボは飛行制御面の割り当てと競合しないよう、`MAV_CMD_DO_SET_ACTUATOR`
（`vehicle_command`）経由で出力する：

```
commandCatapultTailServos()          FunctionActuatorSet              QGC output
  ├─ param1 ──────────────────→  Peripheral_via_Actuator_Set1  →  MAIN 5 (tail servo A)
  └─ param2 ──────────────────→  Peripheral_via_Actuator_Set2  →  MAIN 6 (tail servo B)
  (param7=0, param3..6 = NaN で未変更)
```

- `FunctionActuatorSet` は **index 0（param7=0）のみ**を処理し、param1〜param6 が
  Set1〜Set6 に対応する。
- 正規化値 = `(pwm_us - 1500) / 500`（1000〜2000µs → −1.0〜+1.0）。
- **モーター（MAIN3）は DO_SET_ACTUATOR を使わない**。PX4 通常の推進出力（Motor 1）の
  ままで、カタパルト拡張は `_att_sp.thrust_body[0]`（TECSスロットル）に
  「尾翼解放まで idle 固定」のゲートを掛けるだけ。

### 3.3 変更ファイルと挿入箇所

すべて `src/modules/fw_pos_control/` 配下。

| ファイル | 変更内容 |
|---|---|
| `fw_path_navigation_params.c` | 末尾に `CAT_*` パラメータ7個を追加（group "FW Launch detection"） |
| `FixedwingPositionControl.hpp` | メンバ変数（`_cat_launch_time` 等3個）、`_vehicle_command_pub`、`DEFINE_PARAMETERS` への登録、`commandCatapultTailServos()` 宣言 |
| `FixedwingPositionControl.cpp` | `control_auto_takeoff()` への尾翼ロック/解放・モーターゲート、`commandCatapultTailServos()` 実装、`reset_takeoff_state()` への状態リセット |

`FixedwingPositionControl.cpp` の主な挿入点：

- **尾翼ロック**: カタパルトパス（`else` ブロック）先頭。`CAT_EN && !ロック済み && !launch検知` で
  ロックPWMを一度発行。
- **T0記録**: launch 検知エッジ（`_launch_detected` が立つ箇所）で `_cat_launch_time = now`。
- **尾翼解放**: 上記エッジ処理直後。`CAT_EN && launch検知 && !解放済み && (now-T0) >= CAT_TAIL_DLY` で
  解放PWMを一度発行。
- **モーターゲート**: `max_takeoff_throttle` と `_att_sp.thrust_body[0]` の代入箇所。
  `cat_hold_motor = CAT_EN && CAT_MOT_REQ_TAIL && !解放済み` が真なら、launch detector が
  FLYING でも idle を維持。
- **状態リセット**: `reset_takeoff_state()`（disarm時に呼ばれる）で `_cat_*` を初期化。

### 3.4 設計上の注意

- カタパルト拡張は `vehicle_command` を **publish** する（従来は subscribe のみ）。発行するのは
  `DO_SET_ACTUATOR`(187) で、自身の `vehicle_command_poll()` は `DO_GO_AROUND` /
  `DO_CHANGE_SPEED` しか処理しないため自己フィードバックループは発生しない。
- `CAT_ACC_AXIS`（加速度検知軸の選択）は **実装していない / 不要**。`SENS_BOARD_ROT`
  （FC取り付け向き補正）により body-X 軸が機体前方軸になるため、前方射出のカタパルトでは
  body-X（既存の `_body_acceleration_x`）がそのまま射出軸となる。軸選択が要るのは
  垂直射出など前方射出でない特殊ケースのみ。

---

## 4. パラメータ一覧（Approach A）

`CAT_*` は `fw_path_navigation_params.c` で定義（group: FW Launch detection）。

| パラメータ | 型 | 既定値 | 意味 |
|---|---|---|---|
| `CAT_EN` | bool | 0 | カタパルト拡張の有効化。0=標準挙動 |
| `CAT_TAIL_DLY` | float [s] | 1.0 | launch検知(T0)から尾翼解放までの遅延 |
| `CAT_TAIL5_LCK` | int [µs] | 1000 | MAIN5 ロック位置PWM |
| `CAT_TAIL5_REL` | int [µs] | 2000 | MAIN5 リリース位置PWM |
| `CAT_TAIL6_LCK` | int [µs] | 1000 | MAIN6 ロック位置PWM |
| `CAT_TAIL6_REL` | int [µs] | 2000 | MAIN6 リリース位置PWM |
| `CAT_MOT_REQ_TAIL` | bool | 1 | 1=尾翼解放までモーターをidle保持（筒内暴発防止） |
| `CAT_TO_MODE` | int | 0 | 射出後の自動モード遷移。0=なし/1=Stab/2=Alt/3=Pos/4=Hold/5=Mission/6=Manual |

### `CAT_TO_MODE`（射出後の自動フライトモード遷移）

尾翼解放（射出シーケンス完了）後に、`DO_SET_MODE` で選択モードへ一度だけ自動遷移する。
0なら遷移せず標準挙動（AUTO_TAKEOFF継続→高度到達でHold）。手動系（Stab/Alt/Pos/Manual）は
RC入力が前提、Auto系（Hold/Mission）はRC不要で自律動作。

- 実機でRC操縦に引き継ぐ → `CAT_TO_MODE=1`（Stabilized）
- SITL（RC無し）で射出後の飛行継続を確認 → `CAT_TO_MODE=4`（Hold）

実装: `commandPostLaunchMode()`（`FixedwingPositionControl.cpp`）。尾翼解放後 `_cat_mode_requested`
で一度だけ発行。`reset_takeoff_state()` でリセット。

### 併せて必要な既存パラメータ

| パラメータ | 設定 | 理由 |
|---|---|---|
| `RWTO_TKOFF` | **0** | 滑走離陸を無効化 → カタパルト/ハンドランチのパスに入る（必須） |
| `FW_LAUN_DETCN_ON` | 1 | 加速度による launch 検知を有効化 |
| `FW_LAUN_AC_THLD` | 機体調整 | 射出検知の body-X 加速度しきい値 |
| `FW_LAUN_AC_T` | 機体調整 | しきい値を連続超過する必要時間 |
| `SENS_BOARD_ROT` | 機体調整 | FC取り付け向き補正（body-X を機体前方に合わせる前提） |

---

## 5. ビルド

```bash
# SITL（シミュレーション）
make px4_sitl_default

# 実機（Pixhawk 6C Mini）
make px4_fmu-v6c_default
```

- fmu-v6c の Flash 使用率: **97.58%**（1,918,452 B / 1,966,080 B、空き約47.6KB）。
- 成果物ファーム: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`

### 実機書き込み

```bash
# FCをUSB-Cで直結後
make px4_fmu-v6c_default upload
```

または QGroundControl の "Custom firmware file" で上記 `.px4` を選択して書き込み
（ローカルにビルド環境が無い場合に便利）。

---

## 6. QGC アクチュエータ設定（実機）

実機では QGC で以下を割り当てる（Approach A）：

| 出力 | Function | 備考 |
|---|---|---|
| MAIN 5 | Peripheral via Actuator Set 1 | 尾翼サーボA。PWM Min1000/Max2000 |
| MAIN 6 | Peripheral via Actuator Set 2 | 尾翼サーボB。PWM Min1000/Max2000 |
| MAIN 3 | **Motor 1（変更不要）** | 既存の推進出力のまま |

サーボが逆向きの場合は `CAT_TAIL6_LCK` / `CAT_TAIL6_REL` を反転。
地上テストが済むまで `CAT_EN=0` のままにする。

---

## 7. SITL 動作検証

外部シミュレータ不要の **SIH**（Simulator-In-Hardware）で検証。

```bash
PX4_SIM_MODEL=sihsim_airplane ./build/px4_sitl_default/bin/px4 \
  ./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i 21
```

検証時のポイント：

- 複数起動時は `-i <N>` でインスタンス番号を変え、lock/socket 競合を回避。
- **`RWTO_TKOFF=0` が必須**（SIH airplane は既定で 1 = 滑走離陸のため、カタパルトパスに入らない）。
- SIH には射出加速が無いため、launch 検知は `FW_LAUN_DETCN_ON=0`（即 forceSetFlyState）で
  代替検証した。

### 検証結果（Approach A）

ログで以下のシーケンスを確認：

```
[catapult] tail locked (MAIN5=1000us MAIN6=1000us), waiting for launch
[catapult] tail released at T0+1.00s (MAIN5=1800us MAIN6=1700us)
Takeoff detected            ← 尾翼解放の「後」にモーター始動→離陸
```

- 尾翼ロック → T0+遅延で解放 → モーターゲート解禁 → 離陸、の順序を確認。
- 回帰: `CAT_EN=0` でカタパルトログ無し・標準離陸が正常動作。

### Approach B（PoC）の検証結果（参考）

独立モジュールの6状態 SM（IDLE→ARMED_WAIT→LAUNCHED→TAIL_RELEASED→MOTOR_STARTED→COMPLETE）
を全パス検証済み。`CAT_EN=0` での即時IDLE復帰、モーター自動/手動パスも確認。詳細は
`catapult/approach-b-poc` ブランチ参照。

---

## 8. 既知の制約・今後

- **disarm 経由のアボート**は SITL 未検証（空中での `commander disarm` は commander が
  拒否するため）。`CAT_EN=0` 経由のアボートは検証済み。
- **Flash 残量が少ない**（約47.6KB）。今後モジュールを追加する余地は小さい。
- 実機での射出加速度に合わせた `FW_LAUN_AC_THLD` / `FW_LAUN_AC_T` の実地調整が必要。

---

## 9. 関連ファイル早見

| 種別 | パス |
|---|---|
| Approach A 実装 | `src/modules/fw_pos_control/FixedwingPositionControl.{cpp,hpp}` |
| Approach A パラメータ | `src/modules/fw_pos_control/fw_path_navigation_params.c` |
| Approach B 実装（別ブランチ） | `src/modules/catapult_launch/` |
| 出力関数（参考） | `src/lib/mixer_module/functions/FunctionActuatorSet.hpp` |
| 既存 launch detector | `src/modules/fw_pos_control/launchdetection/LaunchDetector.{cpp,h}` |
