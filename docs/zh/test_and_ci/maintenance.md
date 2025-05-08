# 维护说明

这选择并描述了一些工具来帮助分析代码库的状态并支持其维护。

## 分析 churn

改动的数量，因此对文件所做的更改次数可以指示哪些文件/部件可能需要重构。

To find churn metrics a tool such as [Churn](https://github.com/danmayer/churn) can be used:

```sh
gem install churn
```

An example output as of `v1.6.0-rc2` would be:

```sh
cd src/Firmware
churn --start_date "6 months ago"
**********************************************************************
* Revision Changes
**********************************************************************
Files
+------------------------------------------+
| file                                     |
+------------------------------------------+
| src/modules/navigator/mission.cpp        |
| src/modules/navigator/navigator_main.cpp |
| src/modules/navigator/rtl.cpp            |
+------------------------------------------+



**********************************************************************
```
