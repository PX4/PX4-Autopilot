# Parameters & Configurations

PX4 uses the _param subsystem_ (a flat table of `float` and `int32_t` values) and text files (for startup scripts) to store its configuration.

This section discusses the _param_ subsystem in detail.
매개변수를 나열, 저장 및 로드하는 방법과 매개변수를 정의하고 지상국에서 사용하는 방법을 설명합니다.

:::tip
[System startup](../concept/system_startup.md) and the way that [frame configuration](../dev_airframes/adding_a_new_frame.md) startup scripts work are detailed on other pages.
:::

## 명령줄 사용법

The PX4 [system console](../debug/system_console.md) offers the [param](../modules/modules_command.md#param) tool, which can be used to set parameters, read their value, save them, and export and restore to/from files.

### 매개변수 가져오기 및 설정

The `param show` command lists all system parameters:

```sh
param show
```

To be more selective, a partial parameter name with wildcard "\*" can be used:

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

You can use the `-c` flag to show all parameters that have changed (from their defaults):

```sh
param show -c
```

You can use `param show-for-airframe` to show all parameters that have changed from their defaults for just the current airframe's definition file (and defaults it imports).

### 매개변수 내보내기 및 로드

You can save any parameters that have been _changed_ (that are different from airframe defaults).

The standard `param save` command will store the parameters in the current default file:

```sh
param save
```

인수가 제공되면, 이 새 위치 대신 매개변수를 저장합니다.

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to _load_ parameters:

- `param load` first does a full reset of all parameters to their defaults, and then overwrites parameter values with any values stored in the file.
- `param import` just overwrites parameter values with the values from the file and then saves the result (i.e. effectively calls `param save`).

The `load` effectively resets the parameters to the state when the parameters were saved (we say "effectively" because any parameters saved in the file will be updated, but other parameters may have different firmware-defined default values than when the parameters file was created).

By contrast, `import` merges the parameters in the file with the current state of the vehicle.
예를 들어, 시스템 설정의 나머지 부분을 덮어쓰지 않고 보정 데이터가 포함된 매개변수 파일을 가져오는 데 사용할 수 있습니다.

두 경우에 대한 예를 아래에서 설명합니다.

```sh
# 파일을 저장하고 나면 매개변수 값 초기화
param load /fs/microsd/vtol_param_backup
# 추가로 매개변수 값 저장 (불러온다고 해서 자동으로 끝나지는 않음)
param save
```

```sh
# 현재 매개변수 값 목록에 저장한 매개변수 값 병합
param import /fs/microsd/vtol_param_backup
```

## 매개변수 생성/정의

매개변수 정의에는 두 부분이 있습니다.

- [Parameter metadata](#parameter-metadata) specifies the default value for each parameter in firmware along with other metadata for presentation (and editing) of parameters in ground control stations and documentation.
- [C/C++ Code](#c-c-api) that provides access to get and/or subscribe to parameter values from within PX4 modules and drivers.

메타데이터와 코드를 작성하기 위한 몇 가지 접근 방식을 아래에서 설명합니다.
Where possible code should use newer [YAML metadata](#yaml-metadata) and [C++ API](#c-api) over the older C parameter/code definitions, as these are more flexible and robust.

Parameter metadata is [compiled into the firmware](#publishing-parameter-metadata-to-a-gcs),
and made available to ground stations via the [MAVLink Component Information service](https://mavlink.io/en/services/component_information.html).

### 매개변수 이름

매개변수 이름은 ASCII 문자 16자 이하입니다.

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed-wing systems.
이 관례는 강제 사항은 아닙니다.

The name must match in both code and [parameter metadata](#parameter-metadata) to correctly associate the parameter with its metadata (including default value in Firmware).

### C / C++ API

PX4 모듈 및 드라이버에서 매개변수 사용할 수 있는 C 및 C++ API가 있습니다.

API 간의 중요한 차이점 중 하나는 C++ 버전이 매개변수 값(예: GCS에서) 변경과 동기화하는 보다 효율적인 표준화 메커니즘이 있다는 것입니다.

매개변수는 언제든지 다른 값으로 변경될 수 있으므로, 동기화가 중요합니다.
Your code should _always_ use the current value from the parameter store.
If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).

또한, C++ 버전은 유형 안전성이 더 우수하고 메모리 사용량이 적습니다.
단점은 매개변수 이름을 컴파일 타임에 알아야 하는 반면에, C API는 동적으로 생성된 이름을 문자열로 사용할 수 있습니다.

#### C++ API

The C++ API provides macros to declare parameters as _class attributes_.
You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with _any_ parameter update.
그런 다음, 프레임워크 코드는 매개변수 속성에 영향을 미치는 uORB 메시지 추적을 처리하고 동기화 상태를 유지합니다.
나머지 코드에서는 정의된 매개변수 속성을 사용할 수 있으며, 항상 최신 상태를 유지합니다!

제일 먼저, 모듈 또는 드라이버의 클래스 헤더에 필요한 필수 헤더를 포함합니다.

- **px4_platform_common/module_params.h** to get the `DEFINE_PARAMETERS` macro:

 ```cpp
 #include <px4_platform_common/module_params.h>
 ```

- **parameter_update.h** to access the uORB `parameter_update` message:

 ```cpp
 #include <uORB/topics/parameter_update.h>
 ```

- **Subscription.hpp** for the uORB C++ subscription API:

 ```cpp
 #include <uORB/Subscription.hpp>
 ```

Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes.
매개변수의 이름은 매개변수 메타데이터 정의와 동일하여야 합니다.

```cpp
class MyModule : ..., public ModuleParams
{
public:
	...

private:

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
		(ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
```

매개변수 업데이트와 관련된 uORB 메시지를 확인하기 위해 상용구로 cpp 파일을 업데이트합니다.

Call `parameters_update();` periodically in code to check if there has been an update:

```cpp
void Module::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
```

위의 함수에서 :

- `_parameter_update_sub.updated()` tells us if there is _any_ update to the `param_update` uORB message (but not what parameter is affected).
- If there has been "some" parameter updated, we copy the update into a `parameter_update_s` (`param_update`), to clear the pending update.
- Then we call `ModuleParams::updateParams()`.
 This "under the hood" updates all parameter attributes listed in our `DEFINE_PARAMETERS` list.

The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.

:::tip
The [Application/Module Template](../modules/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter-metadata).
:::

#### C API

C API는 모듈과 드라이버 모두에서 사용할 수 있습니다.

먼저 매개변수 API를 포함합니다.

```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`.
The variable `my_param` can then be used in your module code.

```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

:::info
If `PARAM_NAME` was declared in parameter metadata then its default value will be set, and the above call to find the parameter should always succeed.
:::

`param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`.
If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed

```cpp
# Get the handle to the parameter
param_t my_param_handle = PARAM_INVALID;
my_param_handle = param_find("PARAM_NAME");

# Query the value of the parameter when needed
int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```

### 매개변수 메타데이터

PX4는 확장 매개변수 메타데이터 시스템을 사용하여 사용자에게 매개변수를 표시하고 펌웨어의 매개변수들의 기본값을 설정합니다.

:::tip
Correct metadata is critical for good user experience in a ground station.
:::

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible).
일반적으로 연결된 모듈과 함께 저장됩니다.

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced_config/parameter_reference.md) and the parameter information [used by ground stations](#publishing-parameter-metadata-to-a-gcs).

:::warning
After adding a _new_ parameter file you should call `make clean` before building to generate the new parameters (parameter files are added as part of the _cmake_ configure step, which happens for clean builds and if a cmake file is modified).
:::

#### YAML 메타데이터

:::info
At time of writing YAML parameter definitions cannot be used in _libraries_.
:::

YAML meta data is intended as a full replacement for the **.c** definitions.
다중 인스턴스 정의와 같은 새로운 기능과 함께 동일한 메타데이터를 모두 지원합니다.

- The YAML parameter metadata schema is here: [validation/module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml).
- An example of YAML definitions being used can be found in the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml).
- YAML 파일은 다음을 추가하여 cmake 빌드 시스템에 등록됩니다.

 ```cmake
 MODULE_CONFIG
 	module.yaml
 ```

 to the `px4_add_module` section of the `CMakeLists.txt` file of that module.

#### 다중 인스턴스(템플릿) YAML 메타 데이터

Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) (templated parameter code is not supported).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`.
예를 들어 아래는 MY_PARAM_1_RATE, MY_PARAM_2_RATE 등을 생성합니다.

```yaml
MY_PARAM_${i}_RATE:
            description:
                short: Maximum rate for instance ${i}
```

다음 YAML 정의는 시작과 끝 인덱스를 제공합니다.

- `num_instances` (default 1): Number of instances to generate (>=1)
- `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]\`.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml)

#### c 매개변수 메타데이터

The legacy approach for defining parameter metadata is in a file with extension **.c** (at time of writing this is the approach most commonly used in the source tree).

매개변수 메타데이터 섹션은 다음 예와 같습니다.

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

```cpp
/**
 * Acceleration compensation based on GPS
 * velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

The `PARAM_DEFINE_*` macro at the end specifies the type of parameter (`PARAM_DEFINE_FLOAT` or `PARAM_DEFINE_INT32`), the name of the parameter (which must match the name used in code), and the default value in firmware.

주석 블록의 행은 모두 선택 사항이며, 주로 지상국에서 표시 및 편집 옵션을 제어합니다.
The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml)).

```cpp
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overridden by the user>
 * @max <the maximum sane value. Can be overridden by the user>
 * @decimal <the minimum sane value. Can be overridden by the user>
 * @increment <the "ticks" in which this value will increment in the UI>
 * @reboot_required true <add this if changing the param requires a system restart.>
 * @boolean <add this for integer parameters that represent a boolean value>
 * @group <a title for parameters that form a group>
 */
```

## GCS에 매개변수 메타데이터 게시

The parameter metadata JSON file is compiled into firmware (or hosted on the Internet), and made available to ground stations via the [MAVLink Component Metadata service](https://mavlink.io/en/services/component_information.html).
This ensures that metadata is always up-to-date with the code running on the vehicle.

This process is the same as for [events metadata](../concept/events_interface.md#publishing-event-metadata-to-a-gcs).
For more information see [PX4 Metadata (Translation & Publication)](../advanced/px4_metadata.md)

## 추가 정보

- [Finding/Updating Parameters](../advanced_config/parameters.md)
- [Parameter Reference](../advanced_config/parameter_reference.md)
- [Param implementation](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/param.h#L129) (information on `.get()`, `.commit()`, and other methods)
