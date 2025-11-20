import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
import paho.mqtt.client as mqtt
import time
import random
import json
import threading
from collections import deque
from nav2_simple_commander.robot_navigator import BasicNavigator

from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion




class Nav2MqttClient(Node):
    def __init__(self):
        super().__init__('nav2_mqtt_client')
        self.get_logger().info('Nav2MqttClient initialized')
        
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, self.client_id)
        
        # 添加导航队列和锁
        self.nav_queue = deque()
        self.nav_lock = threading.Lock()

        self.run()

        def on_connect(client, userdata, flags, reason_code, properties):
            if reason_code == 0:
                client.subscribe("go2air")
                client.subscribe("webrtc/#")
                self.get_logger().info("MQTT连接成功")
            else:
                self.get_logger().error(f"MQTT连接失败，错误代码: {reason_code}")
                time.sleep(10)
                client.reconnect()

        def on_message(client, userdata, msg):
            data = json.loads(msg.payload.decode())
         
            if data["type"] == "init_pose":
                
                msg = PoseWithCovarianceStamped()
        
                # 设置header
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # 设置位姿
                msg.pose.pose.position.x = data["msg"][0]
                msg.pose.pose.position.y = data["msg"][1]
                msg.pose.pose.orientation.z = data["msg"][2]
                msg.pose.pose.orientation.w = data["msg"][3]
                
                self.publisher_.publish(msg)
            
            elif data["type"] == "single_pose":
                # 将导航请求放入队列
                with self.nav_lock:
                    self.nav_queue.append({
                        'type': 'single_pose',
                        'data': data["msg"]
                    })
                self.get_logger().info("单点导航请求已加入队列")

            elif data["type"] == "multi_pose":
                # 将导航请求放入队列
                with self.nav_lock:
                    self.nav_queue.append({
                        'type': 'multi_pose',
                        'data': data["msg"]
                    })
                self.get_logger().info("多点导航请求已加入队列")

        self.client.on_connect = on_connect
        self.client.on_message = on_message

        self.client.connect("118.31.73.84", 1883, 60)

        self.client.loop_start()

    def run(self):
        self.navigator = BasicNavigator()
        # 创建发布者
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        # 获取位姿
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1, self.get_transform)
        

        

    def process_nav_queue(self):
        """在主线程中处理导航队列"""
        with self.nav_lock:
            if self.nav_queue:
                nav_request = self.nav_queue.popleft()
                
                if nav_request['type'] == 'single_pose':
                    self.execute_single_pose(nav_request['data'])
                elif nav_request['type'] == 'multi_pose':
                    self.execute_multi_pose(nav_request['data'])

    def execute_single_pose(self, data):
        """执行单点导航"""
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(data[0])
            msg.pose.position.y = float(data[1])
            msg.pose.orientation.z = float(data[2])
            msg.pose.orientation.w = float(data[3])
            
            self.get_logger().info("开始执行单点导航")
            self.navigator.goToPose(msg)
            self.get_logger().info("单点导航完成")
        except Exception as e:
            self.get_logger().error(f"单点导航执行失败: {str(e)}")

    def execute_multi_pose(self, data):
        """执行多点导航"""
        try:
            goal_poses = data
            multi_pose = []
            for i in range(len(goal_poses)):
                msg = PoseStamped()
                
                # 设置header
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # 设置位姿
                msg.pose.position.x = float(goal_poses[i]['x'])
                msg.pose.position.y = float(goal_poses[i]['y'])
                msg.pose.orientation.w = float(goal_poses[i]['z'])
                
                multi_pose.append(msg)

            self.get_logger().info("开始执行多点导航")
            self.navigator.followWaypoints(multi_pose)
            self.get_logger().info("多点导航完成")
        except Exception as e:
            self.get_logger().error(f"多点导航执行失败: {str(e)}")

    def get_transform(self):
        self.process_nav_queue()
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=0.5))
            transform = tf.transform
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])

            if self.client:
                self.client.publish("go2air", json.dumps({"type": "pose", 
                                                            "location": [transform.translation.x, 
                                                                         transform.translation.y, 
                                                                         transform.translation.z], 
                                                            
                                                            "rotation":list(rotation_euler)}),qos=0,retain=False)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MqttClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
   
    main()
    