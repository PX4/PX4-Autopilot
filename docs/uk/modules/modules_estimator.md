# Посилання на Модулі: Оцінка

## AttitudeEstimatorQ

Source: [modules/attitude_estimator_q](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/attitude_estimator_q)

### Опис

Оцінювач висоти q.

<a id="AttitudeEstimatorQ_usage"></a>

### Використання

```
AttitudeEstimatorQ <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## airspeed_estimator

Source: [modules/airspeed_selector](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airspeed_selector)

### Опис

Цей модуль надає єдину тему airspeed_validated, яка містить вказану (IAS), калібровану (CAS), справжню повітряну швидкість (TAS) та інформацію, чи є оцінка зараз недійсною і чи ґрунтується на показаннях датчика чи на швидкості на землі мінус швидкість вітру.
Підтримуючи введення декількох «сирих» входів швидкості повітря, цей модуль автоматично перемикається
на коректний датчик у разі виявлення несправності. Для виявлення несправностей, а також для
оцінки масштабного коефіцієнта від IAS до CAS, вона запускає кілька оцінювачів вітру
а також публікує їх.

<a id="airspeed_estimator_usage"></a>

### Використання

```
airspeed_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## ekf2

Source: [modules/ekf2](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2)

### Опис

Оцінювач відношення та позиції за допомогою розширеного фільтра Калмана. Використовується для багатороторних і фіксованих крил.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

<a id="ekf2_usage"></a>

### Використання

```
ekf2 <command> [arguments...]
 Commands:
   start
     [-r]        Enable replay mode

   stop

   status        print status info
     [-v]        verbose (print all states and full covariance matrix)

   select_instance Request switch to new estimator instance
     <instance>  Specify desired estimator instance
```

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/local_position_estimator)

### Опис

Оцінювач відношення та позиції за допомогою розширеного фільтра Калмана.

<a id="local_position_estimator_usage"></a>

### Використання

```
local_position_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## mc_hover_thrust_estimator

Source: [modules/mc_hover_thrust_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_hover_thrust_estimator)

### Опис

<a id="mc_hover_thrust_estimator_usage"></a>

### Використання

```
mc_hover_thrust_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
