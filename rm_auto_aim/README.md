# rm_auto_aim模块

## 1.简介

rm_auto_aim模块是RoboMaster自瞄功能的一个简单实现，为步兵机器人或在其他射击机器人提供自瞄支持，通过摄像头识别敌方装甲板位置，计算出云台所需要转动的角度。

## 2.文件说明

* 主要文件：

  |            文件            |                           功能描述                           |
  | :------------------------: | :----------------------------------------------------------: |
  |    armor_detector.h/cpp    |                        装甲板识别模块                        |
  | simple_auto_aim_algo.h/cpp | 一个简单自瞄算法的实现（包括装甲板识别，位置解算，抛物线弹道计算，预测等等） |
  |    task_auto_aim.h/cpp     |          自瞄任务ROS封装（顶层模块，连接其他模块）           |

*  node文件:

  |          文件          |    功能描述     |
  | :--------------------: | :-------------: |
  | task_auto_aim_node.cpp | 自瞄任务ROS节点 |

* launch文件：

|           文件            |             功能描述             |
| :-----------------------: | :------------------------------: |
| task_auto_aim_test.launch | 使用图片模拟相机测试自瞄任务功能 |

* script文件

|       文件        |                      功能描述                      |
| :---------------: | :------------------------------------------------: |
| scripts/client.py | 设置自瞄任务状态（开始，暂停等）以及自瞄的目标颜色 |

## 3.快速使用

#### 自瞄测试（图片模拟相机）

launch启动

```bash
ros2 launch rm_auto_aim task_auto_aim_test.launch.py 
```

查看相机图形

```bash
ros2 run rqt_image_view rqt_image_view
```

![](res/test.jpg)

使用client.py启动自瞄任务

```bash
ros2 run rm_auto_aim client.py
```

* 按`w`启动，按`q`暂停

debug信息（图像处理中间过程信息/TODO）

![](doc/imgs/test_result.png)

若想取消图像debug信息，可以修改配置文件res/task_auto_aim_config.yaml

```yaml
get_debug : False
```

### 4.自瞄算法说明

* 参考文档[doc/auto_aim_algo.md](doc/auto_aim_algo.md)

### 5.维护者及开源许可证

- Zhenpeng Ge,  zhenpeng.ge@qq.com
- rm_auto_aim is provided under MIT.
