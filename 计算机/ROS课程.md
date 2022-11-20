# ROS课程

## 安装ROS

> Ubuntu20.04， ROS1

使用鱼香：`wget http://fishros.com/install -O fishros && . fishros`

然后按照提示接着输入就行了。

## 测试ROS（小乌龟）

打开一个终端输入`roscore`

打开第二个终端输入`rosrun turtlesim turtlesim_node`

> 启动节点方法`rosrun pkg_name node-name`

打开第三个终端输入`rosrun turtlesim turtle_teleop_key`

然后可以键盘操作小乌龟，就说明没问题了。

可以在打开一个终端输入`rqt_graph`查看节点关系和话题列表。

## 新建工作空间目录

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_worksapce  # 初始化工作空间

cd ..
catkin_make  # 编译工作空间


source devel/setup.bash  # 工作空间环境变量生效

echo $ROS_PACKAGE_PATH  # 检测
```

## 创建功能包

命令

```bash
catkin_creat_pkg <package_name> [depend1] [depend2] [depend3]
```

例如：

```bash
cd catkin_ws/src
catkin_create_pkg learning_communication std_msgs rospy roscpp
# 在src中生成一个learning_communication功能包，其中包括package.xml 和CMakeLists.txt
```

回到工作空间根目录下进行编译和设置环境变量

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

## hello world尝试

```shell
mkdir -p demo/src
cd demo
catkin_make
# 创建工作空间并初始化

cd src
catkin_create_pkg hello_world roscpp rospy std_msgs
# 创建功能包并添加依赖那个hello_world就是一个功能包

# 编辑CMakeList.txt
add_executable(${PROJECT_NAME}_node src/hello.cpp)  # 136行

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)  # 149行

# 进入工作空间并编译
cd demo
catkin_make

# 运行
roscore
cd demo
source  ./devel/setup.bash
rosrun hello_world hello_world_node

```



## rosbag

可以录制操作或者回放操作

 



