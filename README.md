# 开源机器人学 | Open Robotics

![version](https://img.shields.io/badge/version-v0.0.1-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green)

**开源机器人学**是一个帮助机器人学习和研究的开源项目。本项目从2019年7月开始建设，并持续更新与维护中。

本项目当前包含：

- 有助于提高机器人学算法学习与研究效率的python小工具集

到当前为止，本项目内容均来源于作者 [@chenjm1109](https://github.com/chenjm1109) 的个人经验，对于不尽人意之处，还请热心纠正。

## 1 机器人算法小工具说明与示例

「机器人算法小工具」是我在做机器人算法时，为了提高效率而编写的python脚本。这些工具本身并不包含机器人学算法知识，大都只是一些数据分析、绘图小程序。利用这些脚本，便于我更直观地分析学习和设计自己地机器人算法，分析机器人出现问题。

### 目录

1. [可视化：绘制机器人DH坐标系](https://github.com/chenjm1109/open-robotics/tree/main/Toolbox/01_dh_view)
2. [可视化：绘制二连杆机器人工作空间](https://github.com/chenjm1109/open-robotics/tree/main/Toolbox/02_twolink_workspace)
3. 辅助计算：机器人常用位姿表达的转换
4. 可视化：采样法绘制任意机器人的工作空间
5. 辅助计算：位置、速度、加速度数据的相互转换

### 1.1 可视化：绘制机器人DH坐标系

D-H法是研究机器人学经常要用到的工具。这个脚本可以根据给出的D-H表生成对应的3维坐标系结构简图。支持标准DH(SDH)和改进DH(MDH)

#### 标准DH

![](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Toolbox/01_dh_view/dh_example.png)

#### 改进DH

![](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Toolbox/01_dh_view/mdh_example.png)

### 1.2 可视化：绘制二连杆机器人工作空间

给定二连杆机器人的连杆长度、手系和关节运动范围，就可以绘制出工作空间的形状。

![](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Toolbox/02_twolink_workspace/twolink_ws_example.png)


## 维护者

[@chenjm1109](https://github.com/chenjm1109)

## License

[MIT](LICENSE)
