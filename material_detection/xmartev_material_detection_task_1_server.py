import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
project_root = os.path.dirname(current_dir)
models_dir = os.path.join(project_root, "models")
ply_dir = os.path.join(project_root, "models", "3dgs")
os.environ["DISCOVERSE_ASSERT_DIR"] = models_dir
sys.path.insert(0, project_root)

# json_name = "r1_rules.json"
# json_dir = f"{current_dir}/referee_json/{json_name}"
# print(json_dir)
# print(models_dir)

import glfw
import mujoco
import argparse
import threading
import numpy as np
from scipy.spatial.transform import Rotation
from discoverse.robots_env.mmk2_base import MMK2Cfg
from discoverse.examples.ros2.mmk2_ros2 import MMK2ROS2
from discoverse.utils import get_body_tmat, step_func, SimpleStateMachine, get_site_tmat

import rclpy 
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

from referee import Referee

# 场景与仿真配置
cfg = MMK2Cfg()
cfg.mjcf_file_path = os.path.join(models_dir, "mjcf/xmartev_task_1.xml")

cfg.timestep       = 0.003
cfg.decimation     = 2

cfg.init_key = "pick"
cfg.sync     = True
cfg.headless = False
cfg.render_set = {
    "fps"    : 24,
    # "width"  : 1280,
    # "height" : 720
    "width"  : 640,
    "height" : 480,
}


# 更新后的 cfg.gs_model_dict
cfg.gs_model_dict = {
    # 机器人部件模型
    "agv_link": os.path.join(ply_dir, "mmk2", "agv_link.ply"),
    "slide_link": os.path.join(ply_dir, "mmk2", "slide_link.ply"),
    "head_pitch_link": os.path.join(ply_dir, "mmk2", "head_pitch_link.ply"),
    "head_yaw_link": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "lft_arm_base": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "lft_arm_link1": os.path.join(ply_dir, "airbot_play", "link1.ply"),
    "lft_arm_link2": os.path.join(ply_dir, "airbot_play", "link2.ply"),
    "lft_arm_link3": os.path.join(ply_dir, "airbot_play", "link3.ply"),
    "lft_arm_link4": os.path.join(ply_dir, "airbot_play", "link4.ply"),
    "lft_arm_link5": os.path.join(ply_dir, "airbot_play", "link5.ply"),
    "lft_arm_link6": os.path.join(ply_dir, "airbot_play", "link6.ply"),
    "lft_finger_left_link": os.path.join(ply_dir, "airbot_play", "left.ply"),
    "lft_finger_right_link": os.path.join(ply_dir, "airbot_play", "right.ply"),
    "rgt_arm_base": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "rgt_arm_link1": os.path.join(ply_dir, "airbot_play", "link1.ply"),
    "rgt_arm_link2": os.path.join(ply_dir, "airbot_play", "link2.ply"),
    "rgt_arm_link3": os.path.join(ply_dir, "airbot_play", "link3.ply"),
    "rgt_arm_link4": os.path.join(ply_dir, "airbot_play", "link4.ply"),
    "rgt_arm_link5": os.path.join(ply_dir, "airbot_play", "link5.ply"),
    "rgt_arm_link6": os.path.join(ply_dir, "airbot_play", "link6.ply"),
    "rgt_finger_left_link": os.path.join(ply_dir, "airbot_play", "left.ply"),
    "rgt_finger_right_link": os.path.join(ply_dir, "airbot_play", "right.ply"),
    # 场景物体模型
    "background": os.path.join(ply_dir, "xmartev_material_detection", "scene.ply"),
    "toolbox": os.path.join(ply_dir, "xmartev_material_detection", "toolbox.ply"),
    "brownbox": os.path.join(ply_dir, "xmartev_material_detection", "brownbox.ply"),
    "blue": os.path.join(ply_dir, "xmartev_material_detection", "blue.ply")
}

cfg.obj_list = [
    # 机器人部件
    "agv_link", "slide_link", "head_pitch_link", "head_yaw_link",
    "lft_arm_base", "lft_arm_link1",
    "lft_arm_link2", "lft_arm_link3", "lft_arm_link4",
    "lft_arm_link5", "lft_arm_link6",  # 添加 lft_arm_link5

    # 左手手指关节2个
    "lft_finger_left_link", "lft_finger_right_link",

    "rgt_arm_base", "rgt_arm_link1",
    "rgt_arm_link2", "rgt_arm_link3", "rgt_arm_link4",
    "rgt_arm_link5", "rgt_arm_link6",  # 添加 rgt_arm_link5

    # 右手手指关节2个
    "rgt_finger_left_link", "rgt_finger_right_link",

    "toolbox", "brownbox", "blue"          
]


# 传感器配置
cfg.obs_rgb_cam_id = [0, 1, 2]  # 保留RGB相机
cfg.obs_depth_cam_id = [0]      # 保留深度相机
cfg.use_gaussian_renderer = True  # 启用高斯渲染
cfg.lidar_s2_sim = True  # 启用激光雷达仿真

class SceneROS2Node(MMK2ROS2):
    def __init__(self, config):
        super().__init__(config)
        # 初始化时钟发布器
        self.clock_publisher_ = self.create_publisher(Clock, '/clock', 10)
        timer_period = 0.01  # 100Hz时钟发布
        self.clock_timer = self.create_timer(timer_period, self.timer_callback)        

        self.round_id = self.config.round_id
        # new
        json_name = f"task_1_rules.json"
        json_dir = f"{current_dir}/referee_json/{json_name}"
             
        self.referee = Referee(self.mj_model, os.path.join(json_dir))
        

    # 时钟发布回调
    def timer_callback(self):
        msg = Clock()
        msg.clock.sec = int(self.mj_data.time)
        msg.clock.nanosec = int((self.mj_data.time - int(self.mj_data.time)) * 1e9)
        self.clock_publisher_.publish(msg)

    # 模型加载后初始化（移除任务相关逻辑）
    def post_load_mjcf(self):
        super().post_load_mjcf()
        # 仅保留基础场景初始化，移除任务道具信息


    def domain_randomization(self):
        
       # # # 随机 第四层物体x,y位置
        self.mj_data.qpos[self.njq+0] += 2. *(np.random.random()-0.5) * 0.03
        # self.mj_data.qpos[self.njq+1] += 2.*(np.random.random()-0.5) * 0.025

        # # # # # # 随机 第三层物体x,y位置
        self.mj_data.qpos[self.njq+7*1+0] += 2. *(np.random.random()-0.5) * 0.03
        # self.mj_data.qpos[self.njq+7*1+1] += 2.*(np.random.random()-0.5) * 0.025


        # # # # # # 随机 第二层物体x,y位置
        self.mj_data.qpos[self.njq+7*2+0] += 2. *(np.random.random()-0.5) * 0.03
        # self.mj_data.qpos[self.njq+7*2+1] += 2.*(np.random.random()-0.5) * 0.025

        height_choice = [0.568601, 0.891876, 1.23]

        # # 随机选择三个不同的值
        selected_heights = np.random.choice(height_choice, size=3, replace=False)

        # # 将选择的值分别赋给棉包的高度
        self.mj_data.qpos[self.njq+2], self.mj_data.qpos[self.njq+7*1+2], self.mj_data.qpos[self.njq+7*2+2] = selected_heights


    # 重置场景（保留基础重置逻辑）
    def reset(self, fix):
        ret = super().reset()
        # 移除任务相关的道具位置随机化和任务初始化
        mujoco.mj_forward(self.mj_model, self.mj_data)
        r = f"round{self.round_id}"
        task_info_msg = String()

        task_info_msg.data = f"{r}: Take the brownbox from the cabinet, and put it on the table."
        
        print(f"{task_info_msg.data}")
        if not fix:
            self.domain_randomization()
        return ret

    # 按键事件处理（移除任务相关功能）
    def on_key(self, window, key, scancode, action, mods):
        super().on_key(window, key, scancode, action, mods)
        # 保留基础按键功能，移除任务相关按键逻辑

    # 物理步后处理（移除任务评分逻辑）
    def post_physics_step(self):
        # 仅保留必要的场景更新，移除所有任务检查逻辑
        self.referee.update(self.mj_data)

        # print("哈哈")
        # breakpoint() 
        # pass

    # 终止条件检查（始终返回False，保持场景运行）
    def checkTerminated(self):
        return False

    # 发布ROS2话题（保留所有传感器数据发布）
    def pubRos2TopicOnce(self):
        time_stamp = self.get_clock().now().to_msg()
        # 发布关节状态
        self.joint_state.header.stamp = time_stamp
        self.joint_state.position = self.sensor_qpos[2:].tolist()
        self.joint_state.velocity = self.sensor_qvel[2:].tolist()
        self.joint_state.effort = self.sensor_force[2:].tolist()
        self.joint_state_puber.publish(self.joint_state)

        # 发布里程计信息
        self.odom_msg.pose.pose.position.x = self.sensor_base_position[0]
        self.odom_msg.pose.pose.position.y = self.sensor_base_position[1]
        self.odom_msg.pose.pose.position.z = self.sensor_base_position[2]
        self.odom_msg.pose.pose.orientation.w = self.sensor_base_orientation[0]
        self.odom_msg.pose.pose.orientation.x = self.sensor_base_orientation[1]
        self.odom_msg.pose.pose.orientation.y = self.sensor_base_orientation[2]
        self.odom_msg.pose.pose.orientation.z = self.sensor_base_orientation[3]
        self.odom_puber.publish(self.odom_msg)

        # 发布头部相机RGB图像
        head_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][0], encoding="rgb8")
        head_color_img_msg.header.stamp = time_stamp
        head_color_img_msg.header.frame_id = "head_camera"
        self.head_color_puber.publish(head_color_img_msg)

        # 发布头部相机深度图像
        head_depth_img = np.array(np.clip(self.obs["depth"][0]*1e3, 0, 65535), dtype=np.uint16)
        head_depth_img_msg = self.bridge.cv2_to_imgmsg(head_depth_img, encoding="mono16")
        head_depth_img_msg.header.stamp = time_stamp
        head_depth_img_msg.header.frame_id = "head_camera"
        self.head_depth_puber.publish(head_depth_img_msg)
        
        # 发布左侧相机RGB图像
        left_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][1], encoding="rgb8")
        left_color_img_msg.header.stamp = time_stamp
        left_color_img_msg.header.frame_id = "left_camera"
        self.left_color_puber.publish(left_color_img_msg)

        # 发布右侧相机RGB图像
        right_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][2], encoding="rgb8")
        right_color_img_msg.header.stamp = time_stamp
        right_color_img_msg.header.frame_id = "right_camera"
        self.right_color_puber.publish(right_color_img_msg)



    def thread_pubGameInfo(self, freq=1):
        rate1 = self.create_rate(freq)
        r = f"round{self.round_id}"

        self.task_info_puber = self.create_publisher(String, '/xmartev_material_detection/taskinfo', 2)
        task_info_msg = String()

        task_info_msg.data = f"{r}: Take the brownbox from the cabinet, and put it on the table."
        

        self.game_info_puber = self.create_publisher(String, '/xmartev_material_detection/gameinfo', 2)

        while rclpy.ok() and self.running:

            self.task_info_puber.publish(task_info_msg)
            self.game_info_puber.publish(self.referee.game_info_msg)
            

            # print(task_info_msg.data)
            # print(self.referee.game_info_msg.data)

            rate1.sleep()


if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
 
    # 简化参数解析（仅保留必要配置）
    parser = argparse.ArgumentParser(description='ROS2 Scene Server for xmartev_material_detection task_1 Environment')
    parser.add_argument('--round_id', type=int, choices=[1, 2, 3], help='tasks round index', required=True)
    parser.add_argument('--fix', type=int, choices=[0,1], help='test_baseline',  default=False,required=False)
    args = parser.parse_args()

    cfg.round_id = args.round_id

    sim_node = SceneROS2Node(cfg)
  
    obs = sim_node.reset(args.fix)

    # 启动ROS2自旋线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(sim_node))
    spin_thread.start()

    # 启动激光雷达发布线程
    publidar_thread = threading.Thread(target=sim_node.thread_publidartopic, args=(12,))
    publidar_thread.start()

    # 启动传感器话题发布线程
    pubtopic_thread = threading.Thread(target=sim_node.thread_pubros2topic, args=(24,))
    pubtopic_thread.start()


    pubgameinfo_thread = threading.Thread(target=sim_node.thread_pubGameInfo)
    pubgameinfo_thread.start()

    try:
        # 主仿真循环
        while rclpy.ok() and sim_node.running:
            obs, _, _, ter, _ = sim_node.step(sim_node.target_control)

            if ter:
                print(f"get final score:{sim_node.referee.total_score[0]}")
                sim_node.running = False
                break

            sim_time = sim_node.mj_data.time
            if sim_time >= 180:
                print(f"get final score:{sim_node.referee.total_score[0]}")
                sim_node.running = False
                break

    except KeyboardInterrupt:
        pass

    finally:
        # 资源清理
        publidar_thread.join()
        pubtopic_thread.join()
        sim_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()
        print("Scene ROS2 Server shutdown complete")
