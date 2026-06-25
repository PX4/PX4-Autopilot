# ElasticCatapultSystem — Gazebo カタパルト射出シミュレーション

PX4 SITL（Gazebo Harmonic / gz-sim）上で固定翼機のカタパルト射出を再現する
System plugin。カスタム PX4 ファームウェア（Approach A: カタパルト離陸自動化）の
動作検証に使う。

- plugin 本体: `ElasticCatapultSystem.cpp` / `.hpp`
- 適用モデル: `Tools/simulation/gz/models/rc_cessna/model.sdf`（`<plugin>` 追加済み）
- 状態遷移: `WAITING → CATAPULTING → RELEASED`

---

## 1. 検証結果（2026-06-24）

`gz_rc_cessna` + カスタムFW で、射出から飛行移行までエンドツーエンドで確認済み。

| 確認項目 | 結果 |
|---|---|
| 射出物理の再現 | ✅ リリース速度 **10.73 m/s** |
| FW が射出加速度を検知 | ✅ `Takeoff detected`（launch detector 発火） |
| 尾翼ロック → 解放（CAT_TAIL_DLY=1s後） | ✅ `tail locked` → `tail released at T0+1.00s (MAIN5=2000 MAIN6=2000)` |
| モーターゲート（解放まで idle） | ✅ `CAT_MOT_REQ_TAIL` 動作 |
| リリース後の飛行継続 | ✅ `Landing detected` 0 / `Disarmed` 0 |
| failsafe・異常・クラッシュ | ✅ 0件 |

観測されたログ（抜粋）:

```
INFO  [commander] Armed by ...
INFO  [fw_pos_control] [catapult] tail locked (MAIN5=1000us MAIN6=1000us), waiting for launch
INFO  [commander] Takeoff detected
INFO  [fw_pos_control] [catapult] tail released at T0+1.00s (MAIN5=2000us MAIN6=2000us)
[ElasticCatapult] t=21s  CATAPULTING -> RELEASED  release_speed=10.7298m/s (target=20)
```

### 既知の制約

- リリース速度が目標(20)の約半分(10.7)になるのは、**地上スポーンの車輪摩擦が射出力の
  約半分を吸収**するため。解放後に motor ungate で加速し、着陸・failsafe とも出ず飛行を
  継続するため検証目的は達成。完全な20m/s射出には空中スポーンが必要だが、その場合 PX4 が
  離陸パスに入らず尾翼ロジックが起動しない（**地上スポーンが本検証に最適**）。
- 描画センサーを持つ `advanced_plane` はヘッドレスで Ogre2 がクラッシュするため使えない。
  描画センサーの無い **`rc_cessna`** を使う。

---

## 2. ビルド

```bash
cd PX4-Autopilot
make px4_sitl_default          # FW + gz_bridge + gz_plugins(ElasticCatapult含む) を一括
```

plugin だけ作り直す場合も同じコマンドで差分ビルドされる。
生成物: `build/px4_sitl_default/src/modules/simulation/gz_plugins/libElasticCatapultSystem.so`
（`GZ_SIM_SYSTEM_PLUGIN_PATH` に自動配置される）

---

## 3. SITL の回し方（自分で実行する手順）

### 3.1 重要な前提（このマシン特有）

- **インスタンス番号は `-i 1`**。`-i 0`（既定）は別ユーザーが `/tmp/px4-sock-0` を
  握っていて使えない。
- **ヘッドレス必須**（`HEADLESS=1`）。GUI/GPU が無い環境のため。
- **`GZ_VERBOSE=3`** にしないと plugin のログ（gzmsg）が出ない。

### 3.2 起動コマンド（コピペで動く）

```bash
cd PX4-Autopilot

# gz 環境変数を読み込む（プラグインパス・モデルパス等）
source build/px4_sitl_default/rootfs/gz_env.sh

export GZ_PARTITION="mycat"     # 他のgzと混線しないよう任意の固有名
export GZ_VERBOSE=3             # plugin ログを表示

# PX4 へ流すコマンドを時系列で投入しつつ起動。
# ( sleep ... | px4 ) で stdin を開いたまま保持（プロンプト暴走/デーモン化を防ぐ）
(
  sleep 16
  echo "param set CAT_EN 1"
  echo "param set RWTO_TKOFF 0"
  echo "param set FW_LAUN_DETCN_ON 1"
  echo "param set FW_LAUN_AC_THLD 10"
  echo "param set FW_LAUN_AC_T 0.05"
  echo "param set NAV_DLL_ACT 0"
  echo "param set COM_RCL_EXCEPT 7"
  sleep 2
  echo "commander arm -f"
  sleep 2
  echo "commander takeoff"
  sleep 50
) | PX4_SIM_MODEL=gz_rc_cessna HEADLESS=1 \
    ./build/px4_sitl_default/bin/px4 ./build/px4_sitl_default/etc \
    -s etc/init.d-posix/rcS -i 1 2>&1 | tee /tmp/catapult_run.log
```

> 対話的に自分でコマンドを打ちたい場合は、`( sleep ... ) |` を外して普通に
> `make px4_sitl gz_rc_cessna` 風に起動し、`pxh>` プロンプトで `param set ...`,
> `commander arm -f`, `commander takeoff` を手入力してもよい（ただし `-i 0` 競合に注意）。

### 3.3 結果の確認

```bash
grep -aE "tail locked|Takeoff detected|tail released|ElasticCatapult.*RELEASED" /tmp/catapult_run.log \
  | sed 's/\x1b\[[0-9;]*m//g'
```

### 3.4 終了後のクリーンアップ（重要）

gz サーバーは px4 終了後も残るので必ず掃除する。残っていると次回
「server already running」で失敗する。

```bash
pkill -9 -f "bin/px4"
pkill -9 -f "gz sim.*Tools/simulation/gz/worlds"
rm -f /tmp/px4-sock-1 /tmp/px4_lock-1
```

---

## 4. 変更可能なパラメータ

パラメータは **2系統**：(A) カタパルト plugin（射出の物理）と (B) PX4 ファームウェア
（離陸ロジック）。

### 4.A カタパルト plugin パラメータ（SDF）

**変更場所**: `Tools/simulation/gz/models/rc_cessna/model.sdf` の
`<plugin ... name="custom::ElasticCatapultSystem">` ブロック内の各タグ。
編集後は**再ビルド不要**（SDFは起動時読込）。値を変えて再起動するだけ。

```xml
<plugin filename="libElasticCatapultSystem.so" name="custom::ElasticCatapultSystem">
  <link_name>base_link</link_name>
  <t_start>20.0</t_start>
  <t_catapult>1.0</t_catapult>
  <target_speed>20.0</target_speed>
  <initial_roll_deg>0.0</initial_roll_deg>
  <initial_pitch_deg>10.0</initial_pitch_deg>
  <initial_yaw_deg>0.0</initial_yaw_deg>
  <hold_attitude_until_release>true</hold_attitude_until_release>
  <force_model>constant</force_model>
  <spring_k>80.0</spring_k>
  <spring_x0>1.5</spring_x0>
  <spring_c>0.0</spring_c>
  <catapult_direction_mode>initial_attitude_forward</catapult_direction_mode>
</plugin>
```

| タグ | 意味 | 既定 | 変え方の指針 |
|---|---|---|---|
| `link_name` | 力を加える対象リンク | `base_link` | 通常変更不要 |
| `t_start` | sim開始から射出開始まで[s] | `20.0` | PX4のarm+takeoffが間に合う値に。小さくしすぎると離陸前に射出する |
| `t_catapult` | 射出開始からリリースまで[s] | `1.0` | 短いほど高加速（launch検知しやすい）。長いと緩やか |
| `target_speed` | リリース時の目標速度[m/s] | `20.0` | 機体の失速速度より十分上に |
| `initial_roll_deg` | 初期ロール角[deg] | `0.0` | 射出時の機体姿勢 |
| `initial_pitch_deg` | 初期ピッチ角[deg] | `10.0` | 射出仰角。大きいほど上向きに打ち出す |
| `initial_yaw_deg` | 初期ヨー角[deg] | `0.0` | 射出方位 |
| `hold_attitude_until_release` | リリースまで姿勢拘束するか | `true` | `false`で拘束無し（物理任せ） |
| `force_model` | 射出力モデル | `constant` | **下記参照（重要）** |
| `spring_k` | ばね定数[N/m] | `80.0` | `force_model=spring`時のみ |
| `spring_x0` | 初期伸び[m] | `1.5` | `force_model=spring`時のみ |
| `spring_c` | 減衰係数 | `0.0` | `force_model=spring`時のみ |
| `catapult_direction_mode` | 力の方向 | `initial_attitude_forward` | 初期姿勢の前方。通常変更不要 |

#### `force_model` の選択（最重要）

| 値 | 挙動 | IMU加速度 | リリース速度 | 用途 |
|---|---|---|---|---|
| `constant` | 一定外力 `F=m·target/t_catapult` を物理ソルバ経由で印加 | **検知される** | 地上摩擦で目標の約半分 | **FW検証（推奨）**。launch detectorを発火させたいならこれ |
| `spring` | ばね弾性力（初期最大→リリース0） | 検知される | 設定依存 | ゴム射出を模擬したい場合 |
| `velocity` | 速度を直接ランプ（運動学的） | **検知されない** | 目標どおり正確 | 純粋に速度プロファイルだけ見たい場合。**FWのlaunch検知はしない** |

> **なぜ `constant` を使うか**: PX4 の launch detector は IMU の加速度（比力）で射出を検知する。
> `velocity` は速度を直接書き換えるため IMU が加速度を感じず、FWの尾翼解放ロジックが起動しない。
> FW検証では必ず `constant`（または `spring`）を使うこと。

### 4.B PX4 ファームウェア パラメータ

**変更場所**: 起動コマンドの `echo "param set ..."` 行、または `pxh>` プロンプトで直接、
または QGC の Parameters 画面。

| パラメータ | 検証時の値 | 意味 | 備考 |
|---|---|---|---|
| `CAT_EN` | `1` | カタパルト拡張の有効化 | 0で標準挙動 |
| `RWTO_TKOFF` | `0` | 滑走離陸を無効化 | **0必須**。1だとカタパルトパスに入らない |
| `FW_LAUN_DETCN_ON` | `1` | launch検知の有効化 | |
| `FW_LAUN_AC_THLD` | `10` | 射出検知の加速度しきい値[m/s²] | 射出加速度より低く。高すぎると未検知 |
| `FW_LAUN_AC_T` | `0.05` | しきい値の連続超過時間[s] | 小さいほど検知が早い |
| `CAT_TAIL_DLY` | (既定1.0) | 検知から尾翼解放までの遅延[s] | `param set CAT_TAIL_DLY 1.5` 等 |
| `CAT_TAIL5_LCK/REL` | (既定1000/2000) | MAIN5 ロック/解放PWM[µs] | サーボ向きに合わせ調整 |
| `CAT_TAIL6_LCK/REL` | (既定1000/2000) | MAIN6 ロック/解放PWM[µs] | 逆向きなら反転 |
| `CAT_MOT_REQ_TAIL` | (既定1) | 尾翼解放までモーターidle保持 | 0で即モーター許可 |
| `CAT_TO_MODE` | (既定0) | 射出後の自動モード遷移 | 0=なし/1=Stab/2=Alt/3=Pos/4=Hold/5=Mission/6=Manual |
| `NAV_DLL_ACT` | `0` | データリンク喪失時の動作無効化 | SITLでGCS無し時の誤failsafe回避 |
| `COM_RCL_EXCEPT` | `7` | RC喪失を例外扱い | SITLでRC無し時の誤failsafe回避 |

#### `CAT_TO_MODE`（射出後の自動モード遷移）

尾翼解放（射出シーケンス完了）後に、選択したフライトモードへ自動遷移する。
0なら遷移せず標準挙動（AUTO_TAKEOFF継続→高度到達でHold）。

| 値 | 遷移先 | RC必要 | 用途 |
|---|---|---|---|
| 0 | なし（既定） | — | 標準のAUTO_TAKEOFF |
| 1 | Stabilized | ✅ | 実機RC手動操縦 |
| 2 | Altitude | ✅ | 高度保持＋手動 |
| 3 | Position | ✅ | 位置保持＋手動 |
| 4 | Hold (Loiter) | ❌ | 自律旋回（**SITL検証向き**） |
| 5 | Mission | ❌ | ミッション自動実行 |
| 6 | Manual | ✅ | 完全手動 |

> SITLでRC無しなら `CAT_TO_MODE=4`（Hold）が射出後も自律飛行を続けて確認しやすい。
> 実機でRC操縦に引き継ぐなら `CAT_TO_MODE=1`（Stabilized）。手動系はRC入力が無いと落ちる。

> `CAT_*` の全リストと意味は `CATAPULT_LAUNCH.md`（リポジトリ直下）を参照。

### 4.C よくある変更例

- **射出をもっと強く/速く**: SDF `t_catapult` を `0.5` に（同target_speedなら加速2倍→検知しやすい）
- **射出仰角を変える**: SDF `initial_pitch_deg` を `15` 等に
- **射出タイミングを早める**: SDF `t_start` を下げる（ただしPX4のarm+takeoffが間に合う範囲で）
- **尾翼解放を遅らせる**: `param set CAT_TAIL_DLY 2.0`
- **検知感度を上げる**: `param set FW_LAUN_AC_THLD 5`

---

## 5. トラブルシューティング

| 症状 | 原因 | 対処 |
|---|---|---|
| `server already running for instance 1` | 前回の px4/gz が残存 | §3.4 のクリーンアップを実行 |
| plugin ログ（`[ElasticCatapult]`）が出ない | `GZ_VERBOSE` が低い | `export GZ_VERBOSE=3` |
| Ogre2 / RenderEngine クラッシュ | 描画センサー付きモデル | `rc_cessna` を使う（`advanced_plane`不可） |
| `tail released` が出ない（`tail locked`止まり） | launch未検知 | `force_model=constant`にする / `FW_LAUN_AC_THLD`を下げる |
| ログが巨大化（GB級） | `pxh>`プロンプト暴走 | `( sleep N ) | px4` 形式で stdin を保持 |
| リリース速度がほぼ0 | （対処済み）WAITING中の速度コマンド残留 | 最新版pluginで修正済み |
