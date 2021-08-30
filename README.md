# 开源机器人学 | Open Robotics

![version](https://img.shields.io/badge/version-v0.0.1-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green)

**开源机器人学**是一个帮助机器人学习者入门的开源项目。本项目从2019年7月开始建设，并持续更新与维护中。

本项目当前包含：

1. 有助于提高机器人学的学习与研究效率的python小工具集
2. 课程「机器人算法开发实践」的任务说明与代码示例
3. [「小明工坊」](https://www.zhihu.com/people/xiao-ming-gong-fang/posts)在知乎、B站、古月居等公开平台发表教程的源工程

到当前为止，本项目内容均来源于作者 [@chenjm1109](https://github.com/chenjm1109) 的个人经验，对于不尽人意之处，还请热心纠正。

## 目录

1. [机器人学python小工具说明与示例](#1-机器人学pyhon小工具说明与示例)
2. [机器人算法开发实践](#2-机器人算法开发实践)
3. [机器人技术系列教程使用说明与示例](#3-机器人技术系列教程使用说明与示例)

## 1. 机器人学python小工具说明与示例

「机器人学python小工具」是我在做机器人算法时，为了提高效率而编写的python脚本。这些工具本身并不包含机器人学算法知识，大都只是一些数据分析、绘图小程序。利用这些脚本，便于我更直观地分析学习和设计自己地机器人算法，分析机器人出现问题。

### 目录

1. [可视化：绘制机器人DH坐标系](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Toolbox/01_dh_view)
2. [可视化：绘制二连杆机器人工作空间](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Toolbox/02_twolink_workspace)
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

## 2. 机器人算法开发实践

### 目录

- [第1次任务：实现机器人算法类的基本架构](https://raw.githubusercontent.com/chenjm1109/open-robotics/main/Course/Section01/material.ipynb)
- 第2次任务：机器人运动学算法的实现
- 第3次任务：关节空间线性轨迹规划
- 第4次任务：雅可比矩阵与奇异位形判定
- 第5次任务：动力学与最大电机转矩设计

### 2.1 课程介绍

「机器人算法开发实践」这门课程，是我在[拓斯达科技](http://www.topstarltd.com/about)承担新人培养任务时，为新接触机器人算法的小伙伴们开设的。

在部门新人培养大纲上，我这样描述本门课程的目的：

- 理解机器人的运动学、动力学、运动规划与控制技术的基本概念，利用二连杆机器人案例串联知识点
- 完成编程小任务，实现基本的机器人学算法代码开发，规范编程技巧和代码风格

截至2020年8月，本门课程的内容完全由我单独设计，其中必然有不妥之处。作为 Open Robotics 项目的一部分，如果你有兴趣，十分欢迎参与到课程的建设和优化中，为更多优秀的同学进入机器人领域作一分贡献。

## 3. 机器人技术系列教程使用说明与示例

「机器人技术系列教程」是指我在知乎、B站、古月居等各平台发表过的有关机器人技术的博客文章，这些文章大都有配套的工程源码。坦白说，这些源码的编写时间比较早，质量并不高。我在整理本项目的过程中，正在陆续对相关代码进行优化，这种优化将会更加关注代码的**可读性**而非效率。

我的文章主要分为**ROS仿真**和**机器人运动规划**两个方向，这些内容和前面的部分相互独立，可以根据需求来使用。

### 3.1 ROS仿真系列

### 3.2 机器人运动规划系列

## 维护者

[@chenjm1109](https://github.com/chenjm1109)

## License

[MIT](LICENSE)
