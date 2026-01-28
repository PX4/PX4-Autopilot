# Modules Reference: Template

## module

Source: [templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module)

### Опис

Розділ, що описує наданий функціонал модуля.

Це шаблон для модуля, що працює як завдання у фоновому режимі з функціями start/stop/status.

### Імплементація

Секція, що описує високорівневу реалізацію цього модуля.

### Приклади

CLI usage example:

```
module start -f -p 42
```

### Usage {#module_usage}

```
module <command> [arguments...]
 Commands:
   start
     [-f]        Optional example flag
     [-p <val>]  Optional example parameter
                 default: 0

   stop

   status        print status info
```

## work_item_example

Source: [examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/work_item)

### Опис

Приклад простого модуля, який виконується з черги завдань.

### Usage {#work_item_example_usage}

```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
