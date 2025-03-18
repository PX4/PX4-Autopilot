# 固定翼角速度/姿态控制器调节指南

本指南介绍如何手动调整固定翼PID控制器。
它是为高级用户/专家设计的，因为错误的 PID 调节可能会使您的飞机坠毁。

:::info
[Autotune](../config/autotune_fw.md) is recommended for most users, as it is far faster, easier and provides good tuning for most frames.
建议对自动调整不起作用或必须进行更加的调校的机型进行手工调整。
:::

## 操作前提

- “微调”必须首先配置（在开始PID调校之前）。
  The [Fixed-Wing Trimming Guide](../config_fw/trimming_guide_fixedwing.md) explains how.
- 调节过程中错误的设置增益可能会使姿态控制不稳定。
  A pilot tuning gains should therefore be able to fly and land the plane in [manual](../flight_modes_fw/manual.md) (override) control.
- 过高地增益(和快速的舵面响应)可能会超过你的机体结构允许最大过载――增加增益时需谨慎。
- 滚转和俯仰参数调整都遵循相同的顺序。
  The only difference is that pitch is more sensitive to trim offsets, so [trimming](../config_fw/trimming_guide_fixedwing.md) has to be done carefully and integrator gains need more attention to compensate this.

## 建立机型基准线

如果有条件飞手可以手动飞行，最好是通过手动飞行试验确定几个系统核心特性。
为了做到这一点，飞这些科目。
在飞行过程中即使你无法及时在纸上记录全部参数，日志文件在供以后调整也将非常有用。

:::info
All these quantities will be automatically logged.
如果你想在不查看日志的情况下直接调整时才需要使用笔进行记录。

- 在合适的空速下平飞。
  注意油门杆位置和空速（例如：70%-> 0.7 油门，15米/秒空速）。
- 以最大油门和足够的空速爬升10-30秒（例如：12米/秒的空速，在30秒内攀升100米）。
- 以零油门和合理的空速来下滑10-30秒（例如：18米/秒的空速，30秒内下降80米）。
- 滚转杆右侧最大，直到滚转角为60度， 然后滚转杆左侧最大，直到滚转角为负60度为止。
- 俯仰最大45度抬头，俯仰最小负45度低头。

:::

本指南将使用这些参数来设置控制器的部分增益。

## 滚转调参

先调节滚转通道，然后俯仰通道。
滚转更安全，因为不准确只会导致运动，而不会导致掉高。

### 调整前馈增益

调整这个增益前，首先将其他增益设定为最低值 (名义上为0.005, 但请检查参数文档)。

#### 设为最小增益

- [FW_RR_I](../advanced_config/parameter_reference.md#FW_RR_I)
- [FW_RR_P](../advanced_config/parameter_reference.md#FW_RR_P)

#### 待调整增益

- [FW_RR_FF](../advanced_config/parameter_reference.md#FW_RR_FF) - start with a value of 0.4.
  增加该参数值 (每次双倍增加) 直到飞机滚转响应特性良好并到达设置值。
  最后将增益降低20%。

### 调整角速率增益

- [FW_RR_P](../advanced_config/parameter_reference.md#FW_RR_P) - start with a value of 0.06.
  增加这个值 (每次翻倍) 直到系统开始抖动。
  然后将增益降低50%。

### 通过积分器增益调整微调偏置

- [FW_RR_I](../advanced_config/parameter_reference.md#FW_RR_I) - start with a value of 0.01.
  增加这个值(每次翻倍)，直到命令和实际滚转角之间没有偏差(这很可能需要查看日志文件)。

## 俯仰调参

俯仰轴可能需要更多的积分增益和正确的俯仰偏置。

### 调整前馈增益

在调整该增益前，将其他增益项设为最小值。

#### 设为最小增益

- [FW_PR_I](../advanced_config/parameter_reference.md#FW_PR_I)
- [FW_PR_P](../advanced_config/parameter_reference.md#FW_PR_I)

#### 待调整增益

- [FW_PR_FF](../advanced_config/parameter_reference.md#FW_PR_FF) - start with a value of 0.4.
  增加该参数值 (每次翻倍) 直到飞机俯仰响应特性良好并到达设置值。
  最后将增益降低20%。

### 调整角速率增益

- [FW_PR_P](../advanced_config/parameter_reference.md#FW_PR_P) - start with a value of 0.04.
  增加这个值 (每次翻倍) 直到系统开始抖动。
  然后将增益降低50%。

### 通过积分器增益调整微调偏置

- [FW_PR_I](../advanced_config/parameter_reference.md#FW_PR_I) - start with a value of 0.01.
  增加这个值(每次翻倍)，直到命令和实际俯仰角之间没有偏差(这很可能需要查看日志文件)。

## 调整外环时间常数

控制环的整体响应偏软/偏硬可通过时间常数调整。
预设值0.5秒对常规固定翼是个合适的值，通常不需要调整。

- [FW_P_TC](../advanced_config/parameter_reference.md#FW_P_TC) - set to a default of 0.5 seconds, increase to make the Pitch response softer, decrease to make the response harder.
- [FW_R_TC](../advanced_config/parameter_reference.md#FW_R_TC) - set to a default of 0.5 seconds, increase to make the Roll response softer, decrease to make the response harder.

## 其他调整参数

本指南将介绍所有重要参数。
Additional tuning parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md).
