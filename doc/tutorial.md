## Installation

### 安装docker

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

### 安装nvidia driver

推荐使用Software & Updates中Additional Drivers安装，创建镜像和容器前需要检查宿主机的显卡驱动是否正常。

打开终端，输入nvidia-smi检查驱动是否安装成功。

安装 nvidia-docker2

```bash
sudo systemctl --now enable docker

distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 注册 dockerhub

注册dockerhub账号：[dockerhub](https://hub.docker.com/)

登录dockerhub账号

```bash
docker login
```

### Build server

环境依赖：

+   ubuntu >= 20.04
+   cuda >= 11.8
+   显存 >= 6GB
+   空余硬盘空间 >= 80G

#### 从docker hub拉取镜像

```bash
docker pull xmartev/material_detection_server:release_v0
```



### Run server container

打开`scripts/create_container_server.sh`并修改镜像 和 tag名称

![image-20250220193041501](./assets/bash2.png)

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
docker start xmartev_material_detection
```

❗️ 若后续比赛有内容更新，请进入容器中更新

```bash
cd xmartev_material_detection/scripts && bash exec_server.sh
cd xmartev_material_detection
git pull
```

启动比赛。进入server 容器的终端：

```bash
cd /workspace/xmartev_material_detection
python3 s2r_server.py --round_id 1
# round_id 为 [1、2、3] 对应比赛的轮数
```

### Build client

本地构建client docker镜像。

#### 从 docker hub 拉取镜像 

```bash
docker pull xmartev/material_detection_client:release_v0
```


### Run client container

打开`create_container_client.sh`并修改镜像 和 tag名称

![image-20250220211718735](./assets/bash1.png)

创建client container：

```bash
cd xmartev_material_detection/scripts
bash create_container_client.sh
```

终端中进入client container：

```bash
cd xmartev_material_detection/scripts
bash exec_client.sh
```

### Test ros2 communication

>   ❗️ <CLIENT_CONTAINER_ID>要替换成选手自己构建的client docker container的id，可用`docker ps -a`指令查询

```bash
# server --> client 通信测试
(new terminal)
docker exec -it s2r2025_server bash
ros2 topic pub /server_test std_msgs/msg/String "data: 'hello from server'"

(new terminal)
docker exec -it <CLIENT_CONTAINER_ID> bash
# 查看所有活动的topics
ros2 topic list
# 查看topic信息
ros2 topic info /server_test
# 测试订阅server发布的消息
ros2 topic echo /server_test

# client --> server 通信测试
(new terminal)
docker exec -it <CLIENT_CONTAINER_ID> bash
ros2 topic pub /client_test std_msgs/msg/String "data: 'hello from client'"

(new terminal)
docker exec -it s2r2025_server bash
# 测试订阅server发布的消息
ros2 topic echo /client_test
```

## 用手柄遥控MMK2

MMK2（Mobile Manipulation Kit 2）是本次比赛使用的机器人平台，MMK2是人形升降双臂机器人的名字，以下是使用手柄操作仿真环境里的MMK2的操作指南。

```bash
(new terminal) # 启动比赛
docker exec -it s2r2025_server bash
cd /workspace/SIM2REAL-2025/s2r2025
python3 s2r_server.py --round_id 1

(new terminal)
docker exec -it s2r2025_server bash
# 需要先连接手柄
# ls /dev/input/ | grep js
# 如果有js0则说明已经在container中识别到了手柄
ros2 run joy joy_node

(new terminal) 
docker exec -it s2r2025_server bash
cd /workspace/SIM2REAL-2025/s2r2025
# 如果是Logitech类手柄，将/workspace/SIM2REAL-2025/s2r2025/joy_control_test.py
# line 14 `NUM_BUTTON=12` 改成 `NUM_BUTTON=11`
python3 joy_control_test.py
```

手柄操作说明（以XBOX360为例）：

+   左摇杆：控制底盘移动
+   右摇杆：控制头部运动
+   LT左扳机：升降提高
+   RT右扳机：升降降低
+   LB左肩键 （持续按下 控制左侧机械臂）：
    +   方向键 上下：机械臂末端沿x轴平移
    +   方向键 左右：机械臂末端沿y轴平移
    +   左摇杆 上下：机械臂末端沿z轴平移
    +   左摇杆 左右：机械臂末端绕z轴旋转
    +   右摇杆 左右：机械臂末端绕x轴旋转
    +   右摇杆 上下：机械臂末端绕y轴旋转
    +   LT、RT：控制夹爪开合
+   RB左肩键 （持续按下 控制右侧机械臂）：
    +   操作逻辑同LB。LB、RB可同时按下





### 逆运动学

请参考`SIM2REAL-2025/s2r2025/joy_control_test.py` `Ros2JoyCtl`中的`teleopProcess`方法。

机器人的urdf和mesh可在`SIM2REAL-2025/models/mmk2_model`找到。

## 上传client镜像

选手在client中开发算法，开发完成后打包上传至docker hub，由官方拉取后进行测试，测试使用电脑配置为：

```
cpu : 13th Gen Intel Core i7013700KF x24
gpu : GeForce RTX 4090 24G
Memory : 64GB
```

### 1. 新建privtate repo

参赛队伍在自己注册的dockerhub上新建一个private repo，名字为xmartev

![create_repo](./assets/1.png)

![alt text](./assets/2.png)

### 2. 将client镜像push到private repo

将client镜像打上tag(tag名称，参赛队伍可以自定义)，dockerhub_name为dcokerhub的账号名字
```
docker tag xmartev/material_detection_client:example_tagname dockerhub_name/xmartev:tagname 
```
![change_docker_tag](./assets/3.png)

将新tag的client镜像push到private repo
```
docker push dockerhub_name/xmartev:tagname 
```

![docker_push](./assets/4.png)

### 3. 开发比赛任务

根据private repo和tag名字，修改create_container_client.sh里的镜像名和tag

![image-20250220181043385](./assets/bash.png)

运行create_client.sh，创建新容器。

运行exec_client.sh脚本，进入客户端镜像终端开始进行开发工作。

强烈推荐使用 Git 工具进行代码版本管理，以确保代码的安全性和可追溯性。

在 Docker 环境中，您也可以通过 Visual Studio Code（VS Code）安装docker插件 进行高效开发。

### 4. docker commit

本地保存镜像修改内容，使用原有的tag会覆盖之前tag版本的内容
```
docker commit material_detection_client dockerhub_name/xmartev:new_tag
```

![image-20250220181624907](./assets/6.png)

### 5. docker push

通过docker push到private repo保存当前docker镜像到dockerhub
```
docker push dockerhub_name/xmartev:tagename
```

![image-20250220181914426](./assets/7.png)

### 6. 生成访问token

参考连接：[docker token](https://docs.docker.com/docker-hub/access-tokens/)

在需要提交测试的版本时，将dockerhub用户名、docker token由比赛系统提交。

![enter_account_setting](./assets/8.png)

![create_token_pos](./assets/9.png)

![create_token](./assets/10.png)

![token_created](./assets/11.png)
