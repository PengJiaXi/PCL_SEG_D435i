# PCL_SEG_D435i
Cooperation of digital image catch experiment, HUST

PengJiaXi, GuanShangBin

修改记录:  
V1.0-P 添加了D435i的模板类，可以直接获取PCL格式的点云; 修改了原算法的一些细节  

V1.0-G 添加了在输入点云和障碍物点云图中显示障碍物的最小外接矩形框

V1.1-P 添加了D435i的姿态信息获取，但暂未使用

V1.2-P 合并了V1.1和V1.0的更改，解决了部分冲突。注意：尽量不要使用全局变量，PCL_segment.cpp中的全局变量已经作为类变量放进了h文件  

V1.2-G beta 添加了imu参数的获取及通过加速度或角加速度计算欧拉角。但在计算欧拉角若干次后会自动终止，并且同时获取imu和图像信息时 wait_for_frames不能被调用(参考https://github.com/IntelRealSense/librealsense/issues/7858)。

V2.0-P 大修改
1. 使用了非回调式的、稳定的点云和IMU获取方式;
2. 完善了Ransac3d的位姿约束功能，现在分割平面的功能已经比较稳定;
3. 大幅度完善了Rviz的显示方式，加入了相机的模型，通过view_d435i_model.launch打开Rviz;
4. 通过在test_d435i_camera.urdf文件中加入虚拟Link和姿态变换，解决了点云位置不对的问题，其原因是Ros坐标系和Realsense坐标系的姿态不一样，
   注意：以后在Rviz显示的东西，全部都发布到 clouds_link_virtual 这一帧上即可 (令frame_id = "clouds_link_virtual");
5. 把PCLViewer的测试显示方式全部整合到了一个函数中，方便注释掉

V2.1-G  
1. 添加了rviz的显示矩形框的方式;  
2. 修改了欧式聚类的最大和最小size和体素大小;  
3. 矩形框不再将质心距离地面过高的聚类框出。

V2.2-P
优化了参数

V2.3-G  
将计算OBB矩形框的GetOBB函数进行重写
