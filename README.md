# rmoss_contrib

![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

* 更多内容详见[https://robomaster-oss.github.io](https://robomaster-oss.github.io)

**正在开发中，部分功能不稳定。。。**

rmoss_contrib是RoboMasterOSS中的一个基础项目，基于ROS2开发，采用了模块化设计方法，提供多个功能模块。

* rmoss_contrib主要面向任务级别功能实现，如自瞄任务，能量机关任务等，依赖于rmoss_core，此外，不同的功能包有不同的依赖。
* 目前仅支持ROS2 foxy版本

## 1.主要模块

|           模块            |                 功能说明                 |
| :-----------------------: | :--------------------------------------: |
|        rm_auto_aim        |   RoboMaster基础自瞄任务的简单算法实现   |
| rm_power_rune2019（TODO） | RoboMaster2019能量机关任务的简单算法实现 |

## 2.维护者及开源许可证

* gezp zhenpeng.ge@qq.com

* rmoss_contrib is provided under MIT.
