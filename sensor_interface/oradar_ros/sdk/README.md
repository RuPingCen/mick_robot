
# MS200 SDK基本介绍
MS200 SDK是专门为Oradar MS200激光雷达产品设计的软件开发套件。提供易于使用的C/C++风格的API。通过MS200 SDK,用户可以快速连接Oradar MS200激光雷达并接收激光雷达点云数据。
 
# 运行要求
- Linux系统：Ubuntu 14.04 LTS, Ubuntu 16.04 LTS, Ubuntu 18.04 LTS
- Windows 7/10
- C++ 11编译器
- CMake，版本号为3.5或更高

# 编译和安装方法
首先把sdk压缩包解压出来，解压后的文件名为sdk。
Linux下，使用如下命令：
```
cd sdk
mkdir build
cd build
cmake ..    (可使用cmake -DCMAKE_INSTALL_PREFIX=out .. 指定安装目录为当前路径下的out目录)
make
sudo make install
```
Windows下：

(这里以windows10系统，编译器为QT的MinGW为例，需要将编译器安装路径导入系统环境变量中)

按住shift键，鼠标右键，打开powershell
```
cd sdk
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=out ..
mingw32-make -j8
mingw32-make install
```
生成`liboradar_sdk.a`库文件，`blocking_test`和`non-blocking_test`可执行文件


# SDK 主要函数API说明
|函数名称 | 功能介绍|
|---------|---------------|
|Connect| 检查激光雷达串口并打开，创建激光雷达串口读写线程|
|Disconnect| 关闭激光雷达串口读写线程， 关闭串口
|GrabOneScan           |  获取最新一包点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息|
|GrabOneScanBlocking   |  获取最新一包点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息|
|GrabFullScan          |  获取最新一圈点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息|
|GrabFullScanBlocking  |  获取最新一圈点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息|
|GetRotationSpeed      |  获取最新的电机转速|
|SetRotationSpeed      |  设置电机转速|
|GetTimestamp          |  获取最新包的时间戳|
|GetFirmwareVersion    |  获取上下部组固件版本号|
|GetDeviceSN           |  获取雷达设备SN号|
|Activate              |  激光雷达从待机状态进入测距状态|
|Deactive              |  激光雷达从测距状态进入待机状态|


# 示例使用说明
Linux系统下：

将ms200激光雷达设备通过USB转串口线，插入连接到Ubuntu系统，在Ubuntu系统下打开终端，输入 `ls /dev/ttyACM*` 查看串口设备是否接入，若检测到串口设备，则使用 `sudo chmod 777 /dev/ttyACM*` 命令赋予最高权限。
然后执行SDK Samsple,输入如下命令：
```
cd sdk/build
./blocking_test                 #阻塞式获取一圈数据测试程序
```
或者
```
./non-blocking_test             #非阻塞式获取一圈数据测试程序
```
注：如果命令 ls /dev/ttyACM*查看设备，*不是0时，需要在Samsple测试代码中把设备名由/dev/ttyACM0 替换为对应的设备名。(代码中的port_name变量进行修改)

Windwos系统下：

将ms200激光雷达设备通过USB转串口线，插入到windows系统PC上，通过设备管理器查看串口名称，比如`com10`,需要修改示例代码中的port_name变量改为com10，重新编译。
然后双击`blocking_test.exe`或者`non-blocking_test.exe`或者`blocking_c_api_test.exe`可执行程序即可。