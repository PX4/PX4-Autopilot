# Module params: declare, update, forward to _ekf

Reference pattern: `src/modules/ekf2/EKF2.hpp`/`EKF2.cpp` (navputer was copied
from this, still has ekf2's param names — needs its own `module.yaml` +
renamed `DEFINE_PARAMETERS` block, own prefix e.g. `NPUT_`, to avoid
colliding with the real ekf2 module's registered params).

## 1. Where to declare

Two places, always paired:
- `module.yaml` — the param's metadata: name, type, default, min/max, unit,
  description. Generates the actual entry in the firmware's global param
  table (same mechanism as `ZENOH_ENABLE`, `EKF2_RNGBC_CTRL`).
- `DEFINE_PARAMETERS(...)` macro inside the class (`EKF2.hpp:525` onward),
  one line per param:
  ```cpp
  (ParamFloat<px4::params::EKF2_REQ_GPS_H>) _param_ekf2_req_gps_h,
  (ParamBool<px4::params::EKF2_LOG_VERBOSE>) _param_ekf2_log_verbose,
  ```
  Requires the class to inherit `ModuleParams` (`EKF2.hpp:133`).

## 2. How they get updated

Any param change publishes `parameter_update` (uORB). Subscribe
(`EKF2.hpp:400`):
```cpp
uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
```
Check it in `Run()` (`EKF2.cpp:458-464`):
```cpp
if (_parameter_update_sub.updated() || !_callback_registered) {
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);   // clear the update flag
    updateParams();                          // refreshes every ParamX<> member
    ...
}
```
`updateParams()` is inherited from `ModuleParams` — walks every field in
`DEFINE_PARAMETERS` and re-reads its current value from the param system.

## 3. Forwarding to `_ekf` — zero-copy by construction

EKF2 doesn't manually copy values into `_ekf` after each update. Each
`ParamFloat<>`/`ParamInt<>`/`ParamBool<>` member is constructed bound
directly to a field inside the `Ekf`'s own live `Parameters` struct:
```cpp
parameters *_params;   // EKF2.hpp:522 — from _ekf.getParamHandle()
...
_params(_ekf.getParamHandle()),                     // EKF2.cpp:64
_param_ekf2_predict_us(_params->ekf2_predict_us),   // EKF2.cpp:66
_param_ekf2_delay_max(_params->ekf2_delay_max),     // EKF2.cpp:67
```
`Ekf::getParamHandle()` (`estimator_interface.h:157`) is `return &_params;`
— a pointer straight into the Ekf library's internal `parameters` struct
(`EKF/common.h`). Because the wrapper is constructed around that exact
storage location (not its own private storage), `updateParams()` writes the
new value directly into the Ekf's live struct — no explicit forwarding step
anywhere.

This only works for params that already correspond to a field in the Ekf
library's own `Parameters` struct. If custom aiding logic lives entirely in
`navputer` without touching the shared EKF library's internal struct, this
trick doesn't apply — just use `.get()` directly in `navputer.cpp`.

## 4. Custom params — straightforward

1. Add to your own `module.yaml`:
   ```yaml
   module_name: navputer
   parameters:
   - group: Navputer
     definitions:
       NPUT_MY_PARAM:
         description:
           short: My custom param
         type: float
         default: 1.0
   ```
2. Add to `DEFINE_PARAMETERS` in `navputer.hpp`:
   ```cpp
   (ParamFloat<px4::params::NPUT_MY_PARAM>) _param_npute_my_param,
   ```
3. Use as `_param_npute_my_param.get()` in `navputer.cpp`. To reach into the
   `Ekf` instance's own internal logic rather than just `navputer.cpp`,
   either bind it to an existing `Ekf::Parameters` field the same zero-copy
   way, or extend that struct yourself if forking the EKF library for
   navputer's own aiding sources.
