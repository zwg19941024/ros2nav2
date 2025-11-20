import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
import paho.mqtt.client as mqtt
import time
import random
import json
import asyncio
from nav2_simple_commander.robot_navigator import BasicNavigator

from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

from isaac_nav2.realsenseD435 import WebRTCStreamer


class Nav2MqttClient(Node):
    def __init__(self):
        super().__init__('nav2_mqtt_client')
        self.get_logger().info('Nav2MqttClient initialized')
        
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, self.client_id)

        self.run()

        def on_connect(client, userdata, flags, reason_code, properties):
            if reason_code == 0:
                client.subscribe("go2air")
                client.subscribe("webrtc/#")
            else:
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
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.pose.position.x = data["msg"][0]
                msg.pose.position.y = data["msg"][1]
                msg.pose.orientation.z = float(data["msg"][2])
                msg.pose.orientation.w = float(data["msg"][3])
                self.navigator.goToPose(msg)

                       
                    

            elif data["type"] == "multi_pose":
                goal_poses = data["msg"]
                multi_pose=[]
                for i in range(len(goal_poses)):
                    msg = PoseStamped()
                    
                    # 设置header
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'map'
                    
                    # 设置位姿
                    msg.pose.position.x = float(goal_poses[i]['x'])
                    msg.pose.position.y = float(goal_poses[i]['y'])
                    msg.pose.orientation.w  = float(goal_poses[i]['z'])
                    
                    multi_pose.append(msg)

                self.navigator.followWaypoints(multi_pose)

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
        self.timer = self.create_timer(0.5, self.get_transform)
 
        #self.d435 = WebRTCStreamer()
        

    def get_transform(self):
        
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
            self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')


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