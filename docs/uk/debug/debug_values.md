# Надсилання та отримання дебажних значень

Часто під час розробки програмного забезпечення необхідно виводити окремі важливі числа.
This is where the generic `NAMED_VALUE_FLOAT`, `DEBUG` and `DEBUG_VECT` packets of MAVLink come in.

## Відображення між Повідомленнями відлагодження MAVLink та Темами uORB

Повідомлення для налагодження MAVLink перекладаються в/з тем uORB.
Для того щоб надіслати або отримати відлагоджувальне повідомлення MAVLink, вам потрібно відповідно опублікувати або підписатися на відповідну тему.
Ось таблиці, яка узагальнює відповідність між повідомленнями відладки MAVLink та темами uORB:

| Повідомлення MAVLink                                        | Тема uORB                                                 |
| ----------------------------------------------------------- | --------------------------------------------------------- |
| NAMED_VALUE_FLOAT | debug_key_value |
| DEBUG                                                       | debug_value                          |
| DEBUG_VECT                             | debug_vect                           |

## Посібник: Надсилання Стрічок / Плаваючих пар

This tutorial shows how to send the MAVLink message `NAMED_VALUE_FLOAT` using the associated uORB topic `debug_key_value`.

Код для цього посібника доступний тут:

- [Debug Tutorial Code](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp)
- [Enable the tutorial app](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board) by ensuring the MAVLink debug app (**CONFIG_EXAMPLES_PX4_MAVLINK_DEBUG**) is in the config of your board and set set to 'y'.

Все необхідне для налаштування відлагодження публікації - це цей фрагмент коду.
Спочатку додайте файл заголовка:

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
```

Потім рекламуйте тему значення налагодження (одна реклама для різних опублікованих назв достатня).
Поставте це перед вашим головним циклом:

```C
/* advertise debug value */
struct debug_key_value_s dbg;
strncpy(dbg.key, "velx", sizeof(dbg.key));
dbg.value = 0.0f;
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

І навіть надсилання у головному циклі є ще простішим:

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

:::warning
Multiple debug messages must have enough time between their respective publishings for Mavlink to process them.
Це означає, що код повинен чекати між публікацією кількох відлагоджувальних повідомлень або чергувати повідомлення при кожному виклику функції.
:::

Результат у QGroundControl виглядає так на графіку в реальному часі:

![QGC debugvalue plot](../../assets/gcs/qgc-debugval-plot.jpg)

## Посібник: Отримання Стрічок / Плаваючих пар

The following code snippets show how to receive the `velx` debug variable that was sent in the previous tutorial.

First, subscribe to the topic `debug_key_value`:

```C
#include <poll.h>
#include <uORB/topics/debug_key_value.h>

int debug_sub_fd = orb_subscribe(ORB_ID(debug_key_value));
[...]
```

Потім проведіть опитування на тему:

```C
[...]
/* one could wait for multiple topics with this technique, just using one here */
px4_pollfd_struct_t fds[] = {
    { .fd = debug_sub_fd,   .events = POLLIN },
};

while (true) {
    /* wait for debug_key_value for 1000 ms (1 second) */
    int poll_ret = px4_poll(fds, 1, 1000);

    [...]
```

When a new message is available on the `debug_key_value` topic, do not forget to filter it based on its key attribute in order to discard the messages with key different than `velx`:

```C
    [...]
    if (fds[0].revents & POLLIN) {
        /* obtained data for the first file descriptor */
        struct debug_key_value_s dbg;

        /* copy data into local buffer */
        orb_copy(ORB_ID(debug_key_value), debug_sub_fd, &dbg);

        /* filter message based on its key attribute */
        if (strcmp(_sub_debug_vect.get().key, "velx") == 0) {
            PX4_INFO("velx:\t%8.4f", dbg.value);
        }
    }
}
```
