import rclpy
from rclpy.node import Node

from std_msgs.msg import String


from policy import dict1

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'move_arm_string', 10)
        self.publilsher_gripper = self.create_publisher(String, 'gripper_controller_cmd', 10)
        self.main()

    def main(self):
        msg = String()
    
        while rclpy.ok():
            state = str(input("Enter current state: "))
            action = dict1[state]
            value_str = list(action)[0] if action else ""
            print("Action to perform: ", value_str)
            if value_str == "":
                continue
            if value_str[0] == 'B':
                msg.data = "s1"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v1"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[0] == 'I':
                msg.data = "s2"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v1"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[0] == 'T':
                msg.data = "s3"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v1"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            
            if value_str[1] == 'B':
                msg.data = "s1"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v2"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[1] == 'I':
                msg.data = "s2"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v2"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[1] == 'T':
                msg.data = "s3"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v2"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            
            if value_str[2] == 'B':
                msg.data = "s1"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v3"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[2] == 'I':
                msg.data = "s2"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v3"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            elif value_str[2] == 'T':
                msg.data = "s3"
                self.publisher_.publish(msg)
                input("About to close gripper: continue...")
                msg.data= "close"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
                msg.data = "v3"
                self.publisher_.publish(msg)
                input("Continue...")
                msg.data= "open3"
                self.publilsher_gripper.publish(msg)
                input("Continue...")
            msg.data = "home"
            self.publisher_.publish(msg)
            print("Action performed")
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
        
        
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()