# Modules Reference: Template

## module
Source: [templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module)


### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
```
module start -f -p 42
```


<a id="module_usage"></a>
### Usage
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


### Description
Example of a simple module running out of a work queue.


<a id="work_item_example_usage"></a>
### Usage
```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
