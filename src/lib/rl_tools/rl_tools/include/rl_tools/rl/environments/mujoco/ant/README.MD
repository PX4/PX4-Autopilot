Creating the model file to be included in the binary
```
xxd -i ../../../../../../lib/gymnasium/gymnasium/envs/mujoco/assets/ant.xml model.h
```
Renaming `__________________lib_gymnasium_gymnasium_envs_mujoco_assets_ant_xml` and `__________________lib_gymnasium_gymnasium_envs_mujoco_assets_ant_xml_len` to `model_xml` and `model_xml_len` respectively.