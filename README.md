# XMARTEV 机器人控制系统教程

环境依赖：
```
ubuntu >= 20.04
cuda >= 11.8
显存 >= 6GB
空余硬盘空间 >= 80G
```

## server部署指南

### 1. 安装git

打开终端输入
```
sudo apt update
sudo apt install git -y
```
终端输入下面命令git clone到本地
```
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/DISCOVER-Robotics/xmartev_material_detection.git
cd xmartev_material_detection
```
### 2. 安装docker

若本地尚未安装docker：

```bash
cd xmartev_material_detection/scripts
bash docker_install.sh
```

验证docker安装：

```bash
docker --version
```

安装参考链接，[docker install](https://docs.docker.com/engine/install/ubuntu/)。

### 3. 安装nvidia driver

推荐使用Software & Updates中Additional Drivers安装，创建镜像和容器前需要检查宿主机的显卡驱动是否正常。

打开终端，输入nvidia-smi检查驱动是否安装成功。
出现显卡信息和驱动版本信息即为安装成功。
![image-2025022019304150341](doc/assets/nvidiasmi.png)

### 4.安装 nvidia-docker2

```bash
# 1) 启用 Docker
sudo systemctl enable --now docker

# 2) 添加 NVIDIA Container Toolkit 源（使用 keyring，避免 apt-key）
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
 | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -fsSL https://nvidia.github.io/libnvidia-container/stable/$distribution/libnvidia-container.list \
 | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
 | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3) 让 Docker 使用 NVIDIA 运行时（可设为默认）
sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
sudo systemctl restart docker

```

### 5. 注册 dockerhub

注册dockerhub账号：[dockerhub](https://hub.docker.com/) Docker Hub 是一个类似于 GitHub 的平台，只不过它不是存放代码，而是存放 Docker 镜像。选手在client中开发算法，开发完成后打包上传至docker hub，由官方拉取后进行测试。

登录dockerhub账号

```bash
docker login
```

### 6. 从docker hub拉取server镜像

```bash
# 从docker hub拉取
docker pull xmartev/material_detection_server:release_v0

# 如果因为网络问题拉取失败，提供了国内的镜像仓库
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/material_detection_server:release_v0

# 查看是否成功获取 xmartev/material_detection_server 镜像，如果有输出则说明成功拉取到本地
docker images | grep material_detection_server
```

### 7. Run server container

打开[`scripts/create_container_server.sh`](scripts/create_container_server.sh)并修改镜像 和 tag名称（tag名称以最新的版本为准,如按照上面的版本tagname改为release_v0），如果使用国内镜像源拉取，则需要将第15行的`xmartev/`修改成`crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/`

![alt text](doc/assets/20250907-072228.jpg)

创建server container：

```bash
cd xmartev_material_detection/scripts
bash create_container_server.sh
```

终端中进入server container：

```bash
cd xmartev_material_detection/scripts
bash exec_server.sh
```

电脑重启后，需要重新启动容器

```bash
docker start material_detection_server
```
## client部署指南

### 1. 拉取client镜像
```bash
# 从镜像仓库拉取
docker pull xmartev/material_detection_client:release_v0

# 如果因为网络问题拉取失败，提供了国内的镜像仓库
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/material_detection_client:release_v0

# 查看是否成功获取 xmartev/material_detection_client 镜像，如果有输出则说明成功拉取到本地
docker images | grep material_detection_client

```

### 2. 创建Docker容器
打开scripts/create_container_client.sh并修改镜像 和 tag名称，tagname需要修改为实际的最新tag，如按照上面的版本tagname改为release_v0，如果是从国内镜像源拉取，第15行的xmartev/需要修改为国内镜像源名称，例如crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/

 ![alt text](doc/assets/20250907-072220.jpg)

然后执行
```bash
bash create_container_client.sh
# 查看容器是否创建成功
docker ps
```


### 3. 进入Docker容器

```bash
cd xmartev_material_detection/scripts
bash exec_client.sh
```

### 4. 验证ROS2通信

1. 在第一个终端，进入容器并运行发布节点：

   ```bash
   docker exec -it meterial_detection_baseline bash
   ros2 run demo_nodes_cpp talker
   ```

   成功运行会看到类似以下输出：

   ```
   [INFO] [1620000000.000000000] [talker]: Publishing: 'Hello World: 1'
   ```

2. 在第二个终端，在宿主机上设置环境变量并运行接收节点：

   ```bash
   # 安装对应版本的中间件
   sudo apt update
   sudo apt install ros-<distro>-rmw-cyclonedds-cpp
   
   # 设置环境变量（可写入.bashrc或.zshrc中持久化）
   export ROS_DOMAIN_ID=99
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   
   # 运行接收节点
   ros2 run demo_nodes_cpp listener
   ```

   通信正常会看到类似以下输出：

   ```
   [INFO] [1620000000.000000000] [listener]: I heard: 'Hello World: 1'
   ```

3. 如通信异常，检查两边的ROS_DOMAIN_ID是否一致：

   ```bash
   echo $ROS_DOMAIN_ID
   ```

## Material Detection Baseline 使用

### 1. 进入工作目录

```bash
docker exec -it meterial_detection_client bash
cd /workspace/material_detection_client
```

### 2. 运行baseline

> 注意：请提前开启server端服务，并确保ros2话题可以正常传入此baseline容器内

```bash
python3 baseline_round1_seed99.py
```

## ROS2 Topic 含义
```yaml
Subscribed topics:
 * /clock [rosgraph_msgs/msg/Clock] 1 subscriber
	# 仿真时钟
 * /head_camera/aligned_depth_to_color/camera_info [sensor_msgs/msg/CameraInfo] 1 subscriber
 	# mmk2机器人头部深度相机 内参
 * /head_camera/aligned_depth_to_color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# mmk2机器人头部相机的深度图像，和rgb图像对齐，编码格式为mono16，单位毫米
 * /head_camera/color/camera_info [sensor_msgs/msg/CameraInfo] 1 subscriber
 	# mmk2机器人头部rgb相机 内参
 * /head_camera/color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# mmk2机器人头部相机的rgb图像，编码格式rgb8
 * /left_camera/color/camera_info [sensor_msgs/msg/CameraInfo] 1 subscriber
 	# mmk2机器人左手rgb相机 内参
 * /left_camera/color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# mmk2机器人左侧手臂末端相机的rgb图像，编码格式rgb8
 * /right_camera/color/camera_info [sensor_msgs/msg/CameraInfo] 1 subscriber
 	# mmk2机器人右手rgb相机 内参
 * /right_camera/color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# mmk2机器人右侧手臂末端相机的rgb图像，编码格式rgb8
 * /odom [nav_msgs/msg/Odometry] 1 subscriber
 	# mmk2机器人里程计信息
 * /joint_states [sensor_msgs/msg/JointState] 1 subscriber
 	# mmk2机器人全身关节状态量，顺序为 joint_names: [
    # - slide_joint
    # - head_yaw_joint
    # - head_pitch_joint
    # - left_arm_joint1
    # - left_arm_joint2
    # - left_arm_joint3
    # - left_arm_joint4
    # - left_arm_joint5
    # - left_arm_joint6
    # - left_arm_eef_gripper_joint
    # - right_arm_joint1
    # - right_arm_joint2
    # - right_arm_joint3
    # - right_arm_joint4
    # - right_arm_joint5
    # - right_arm_joint6
    # - right_arm_eef_gripper_joint ]
 
Published topics:
 * /cmd_vel [geometry_msgs/msg/Twist] 1 publisher
 	# 控制mmk2底盘移动
 * /head_forward_position_controller/commands [std_msgs/msg/Float64MultiArray] 1 publisher
 	# 控制mmk2头部移动
 * /left_arm_forward_position_controller/commands [std_msgs/msg/Float64MultiArray] 1 publisher
 	# 控制mmk2左臂移动
 * /right_arm_forward_position_controller/commands [std_msgs/msg/Float64MultiArray] 1 publisher
 	# 控制mmk2右臂移动
 * /spine_forward_position_controller/commands [std_msgs/msg/Float64MultiArray] 1 publisher
 	# 控制mmk2升降移动
```

关于baseline进一步的说明，请参考






# 完成开发后上传client镜像

选手在client中开发算法，开发完成后打包上传至docker hub，并打上tag，由官方拉取后进行测试，测试使用电脑配置为：

```
cpu : 13th Gen Intel Core i7013700KF x24
gpu : GeForce RTX 4090 24G
Memory : 64GB
```

### 1. 新建private repo

参赛队伍在自己注册的dockerhub上新建一个private repo，名字为xmartev

![create_repo](doc/assets/1.png)

![alt text](doc/assets/2.png)

### 2. 将client镜像push到private repo

先登录,终端输入
```
docker login -u <dockerhub_name>
```
将client镜像打上tag(tag名称，参赛队伍可以自定义)，dockerhub_name为dockerhub的账号名字
可以先用下面命令查看本地client_name是否存在:

```
docker images | grep client_name
```
```
docker tag xmartev/material_detection_client:example_tagname dockerhub_name/xmartev:tagname 
```
![change_docker_tag](doc/assets/3.png)

将新tag的client镜像push到private repo
```
docker push dockerhub_name/xmartev:tagname 
```

![docker_push](doc/assets/4.png)

### 3. 开发比赛任务

根据private repo和tag名字，修改create_container_client.sh里的镜像名和tag,这里第三航的material_detection_client是container_name

![image-20250220181043385](doc/assets/bash.png)

运行create_client.sh，创建新容器。

运行exec_client.sh脚本，进入客户端镜像终端开始进行开发工作。

强烈推荐使用 Git 工具进行代码版本管理，以确保代码的安全性和可追溯性。

在 Docker 环境中，您也可以通过 Visual Studio Code（VS Code）安装docker插件 进行高效开发。

### 4. 使用 docker commit 保存容器内修改

本地保存镜像修改内容，使用原有的tag会覆盖之前tag版本的内容，如这里我们的container_name为material_detection_client,dockerhub_name为自己docker hub的名字。
```
docker commit container_name dockerhub_name/xmartev:new_tag
```

![image-20250220181624907](doc/assets/6.png)

### 5. docker push（推送新镜像）

通过docker push到private repo保存当前docker镜像到dockerhub
```
docker push dockerhub_name/xmartev:tagename
```

![image-20250220181914426](doc/assets/7.png)

### 6. 生成访问 Token（用于评测拉取）

参考连接：[docker token](https://docs.docker.com/docker-hub/access-tokens/)
>   非常重要：在需要提交测试的版本时，选手需要将自己的dockerhub用户名、docker token 和 镜像的tag由比赛系统提交。以下为生成docker token的指南。
>

![enter_account_setting](doc/assets/8.png)

![create_token_pos](doc/assets/9.png)

![create_token](doc/assets/10.png)

![token_created](doc/assets/11.png)
