# PCL_SEG_D435i
Cooperation of digital image catch experiment, HUST

PengJiaXi, GuanShangBin

修改记录:  
V1.0-P 添加了D435i的模板类，可以直接获取PCL格式的点云; 修改了原算法的一些细节  
V1.0-G 添加了在输入点云和障碍物点云图中显示障碍物的最小外接矩形框
V1.1-P 添加了D435i的姿态信息获取，但暂未使用
V1.2-P 合并了V1.1和V1.0的更改，解决了部分冲突。注意：尽量不要使用全局变量，PCL_segment.cpp中的全局变量已经作为类变量放进了h文件
