# SDK of magnetic interference compensation

### 编译和运行

```sh
tar -zxvf mic_sdk.tar.gz
cd sdk
. ./scripts/build.sh
. ./scripts/set_env.sh
build/mic_test ./models/mic_model_0909.mdl
```

```sh
cd ${WORKING_DIRECTORY}/build
cmake ..
make install -j12
```

### 相关说明

1. 压缩包`mic_sdk.tar.gz`中包含磁干扰补偿软件库（C++），补偿功能接口定义可查阅`./include/api/mic_compensation_api.h`，使用方法示例可参阅`./mic_test.cc`。

2. 模型文件`./models/mic_model_0814.mdl`和`./models/mic_model_0909.mdl` 分别是8月14日和9月9日标定飞行所采集数据生成的磁补偿模型，可用于补偿相同飞机的其他飞行架次中所采集的**舱内**磁测数据；需要注意飞机机身状态、传感器安装情况等必须与标定飞行时相同，若发生变化，需要重新进行标定。

3. 补偿器输出的矢量磁测结果为**惯导坐标系中**的矢量描述，可根据惯导解算的姿态信息，将补偿结果投影到导航坐标系中；若已经惯导与磁强计的坐标转换关系，也可将其投影到磁强计坐标系下。

4. 压缩包`./mic_sdk_arm.tar.gz`为Orin NX中编译生成的版本，可用于机载板上的部署。
