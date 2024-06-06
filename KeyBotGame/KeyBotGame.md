# 代码说明

## 代码结构

由main.cpp调用run.cpp来负责游戏每回合的循环。run()中通过pMap调用map.cpp中的方法，Map类中有player和thief作为成员。utils包含获取随机数、读取输入的工具。

## 继承关系

player和thief均继承了character类，拥有位置，获取位置，移动,改变坐标等基本方法和成员变量。

## 任务实现

* 玩游戏时先输入mapSize（5-40）自定义大小
* 添加两种player：
* ninja 可穿墙（PointSize多添加了PLAYER_IN_WALL类型，移动时对player为ninja特判）
* enderman（末影人）可按f释放技能，当**与thief距离小于10时**交换位置（Map类添加了playerSkill方法）
* 添加了计分板，可一直循环运行，在一局结束后可输入-1退出。（Map类添加静态变量计数，添加getScore等方法）