## Purpose

The idea of this tool is to automate the passing of parameters from gazebo to QGC in order for any changes made to the vehicle parameters in gazebo be easily passed to QGroundControl.

## Requiremetns

Ensure Python 3 is installed.

No additional dependencies are required.

## Usage

1. Place the ROMFS parameter file to be converted in this directory.
2. Run the script:
   ```bash 
   python3 run.py <input_filename> <output_filename.params> <Vehicle-id>
   ```
3. Open QGroundControl and load the generated .params file.

## Functionality

The tool passes the parameters from the gazebo format `param set-default ASPD_BETA_GATE 1.0` to the QGC format `1	1	ASPD_BETA_GATE	1	6`, only lines that are not commented and start with the word param are converted, currently the above example is passed to be `1	1	ASPD_BETA_GATE	1	idk` since the type is not cheked ,'idk' is used as a place holder, and the Component-Id is assumed to be 1. 

## Usability

The current implementation provides a minimal working example. Note that this example is made to use PX4's Advanced Plane as a template and some manual ajustments are still required/encouraged since for example parameters used only in simulation will be converted.

### Key Assumptions:
- The **Component-Id** is assumed to be `1` (since this information is not available).
- The **Type** field is currently not determined and will be set to a placeholder (`idk` for now).
- Only uncommented lines that begin with `param` are processed.

## Limitations
- The tool currently does not check the type of the parameter.
- The Vehicle-Id and Component-Id are assumed to be 1, as this information is not available in the input file.
- Some manual adjustments may be required since parameters use only in simulation will be converted.

## Future Work

The tool, while self-contained, could be expanded into multiple directions.

Future Improvements:
- Implement automatic detection of the Type field.
- Improve handling of Component-Id if additional information becomes available.
- Provide better error handling and logging.
