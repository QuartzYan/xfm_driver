# xfm_driver
## 环境与依赖
- TX2 Ubuntu16.04
- 沥拓A300载板
- xfm10621
- ros-kinetic
- python-smbus2 python-gpio

## 使用
### 1.克隆代码到你的ros工作空间
```shell
cd ~/you_ros_ws/src
git clone https://github.com/QuartzYan/xfm_driver.git
```

### 2.增加i2c，gpio访问权限
```shell
cd xfm_driver/script
./add_i2c_permission.sh	
./creat_gpio_rules.sh
```

### 3.编译
```shell
cd ~/you_ros_ws
catkin_make
```

### 4.运行
```shell
source ./devel/setup.bash
roslaunch xfm_driver bringup.launch
```

## 测试
### 1.回复消息测试
- 打开一个新的终端，运行
```shell
source ~/you_ros_ws/devel/setup.bash
rostopic echo /xfm_status
```
- 唤醒：“灵犀，灵犀”
- 唤醒后终端将会读到，xfm10621的硬件版本信息、唤醒角度、唤醒得分

### 2.模块设置测试
- 打开一个新的终端，运行以下命令，该命令会将xfm10621模块从唤醒模式设置为待唤醒模式，若设置成功，模块的麦克风输出将关闭
```shell
rostopic pub /xfm_cmd xfm_driver/xfm_cmd "xfm_reset:true  xfm_wakeup_disable:false"
```

- 运行以下命令，该命令会将xfm10621模块的唤醒功能关闭，若设置成功，模块将不能通过关键词唤醒
```shell
rostopic pub /xfm_cmd xfm_driver/xfm_cmd "xfm_reset:false  xfm_wakeup_disable:true"
```

- 运行以下命令，该命令会将xfm10621模块的唤醒功能打开，若设置成功，模块将可以通过关键词唤醒
```shell
rostopic pub /xfm_cmd xfm_driver/xfm_cmd "xfm_reset:false  xfm_wakeup_disable:false"
```
