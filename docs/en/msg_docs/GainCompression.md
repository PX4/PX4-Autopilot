---
pageClass: is-wide-page
---

# GainCompression (UORB message)

**TOPICS:** gain_compression

## Fields

| Name                                                    | Type         | Unit [Frame] | Range/Enum | Description                                                         |
| ------------------------------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                     | `uint64`     |              |            | Time since system start (microseconds)                              |
| <a id="fld_compression_gains"></a>compression_gains     | `float32[3]` | [FRD]        | [0 : 1]    | Multiplicative gain to modify the output of the controller per axis |
| <a id="fld_spectral_damper_hpf"></a>spectral_damper_hpf | `float32[3]` | [FRD]        |            | Squared output of spectral damper high-pass filter                  |
| <a id="fld_spectral_damper_out"></a>spectral_damper_out | `float32[3]` | [FRD]        |            | Spectral damper output squared                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GainCompression.msg)

::: details Click here to see original file

```c
uint64 timestamp                # Time since system start (microseconds)

float32[3] compression_gains	# [-] [@frame FRD] [@range 0, 1] Multiplicative gain to modify the output of the controller per axis
float32[3] spectral_damper_hpf  # [-] [@frame FRD] Squared output of spectral damper high-pass filter
float32[3] spectral_damper_out  # [-] [@frame FRD] Spectral damper output squared
```

:::
