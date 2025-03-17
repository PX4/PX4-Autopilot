# 常见问题

## 编译错误

### 闪存溢出

可以加载到主板上的代码量受到其具有的闪存量的限制。
当添加其他模块或代码时，添加可能会超过闪存。
这将导致 "闪存溢出"。 The upstream version will always build, but depending on what a developer adds it might overflow locally.

```sh
region `flash' overflowed by 12456 bytes
```

若要解决此问题，请使用较新的硬件或从生成中删除对您的用例不重要的模块。
The configuration is stored in **/PX4-Autopilot/boards/px4** (e.g. [PX4-Autopilot/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)).
要删除模块，只需将其注释掉：

```cmake
#drivers/trone
```

#### Identifying large memory consumers

The command below will list the largest static allocations:

```sh
sudo apt-get remove modemmanager
```

## USB 错误

### 上传从不成功

On Ubuntu, uninstall the modem manager:

```sh
sudo apt-get remove modemmanager
```
