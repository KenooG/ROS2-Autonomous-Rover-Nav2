#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
import random
import math

class WorldShuffler(Node):
    def __init__(self):
        super().__init__('world_shuffler')
        self.client = self.create_client(SetEntityPose, '/world/mars_mission/set_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Czekam na Gazebo...')

    def shuffle(self):
       
        pos_food = [-5.8618, 99.0651, 1.6335, 1.1627]
        pos_waste = [-100.99, 10.962, 1.9959, -0.0837]

        if random.choice([0, 1]) == 1:
            # SWAP
            res1 = self.send_request('food_box', pos_waste)
            res2 = self.send_request('waste_box', pos_food)
            if res1 and res2:
                self.get_logger().info('BOXES CHANGE LOCATION')
            else:
                self.get_logger().error('FAILED TO MOVE BOXES')
        else:
            # STAY
            res1 = self.send_request('food_box', pos_food)
            res2 = self.send_request('waste_box', pos_waste)
            if res1 and res2:
                self.get_logger().info('BOXES STAY')
            else:
                self.get_logger().error('FAILED TO MOVE BOXES')

    def send_request(self, name, data):
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = 2
        req.pose.position.x = float(data[0])
        req.pose.position.y = float(data[1])
        req.pose.position.z = float(data[2])
        
        yaw = float(data[3])
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success if future.result() else False

def main(args=None):
    rclpy.init(args=args)
    node = WorldShuffler()
    node.shuffle()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()