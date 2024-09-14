import rclpy
from rclpy.node import Node

from std_msgs.msg import String


from policy import dict1

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'move_arm_string', 10)
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
            if value_str[0] == 'b':
                msg.data = "s1"
                self.publisher_.publish(msg)
                msg.data = "v1"
                self.publisher_.publish(msg)
            elif value_str[0] == 'i':
                msg.data = "s2"
                self.publisher_.publish(msg)
                msg.data = "v1"
                self.publisher_.publish(msg)
            elif value_str[0] == 't':
                msg.data = "s3"
                self.publisher_.publish(msg)
                msg.data = "v1"
                self.publisher_.publish(msg)
            
            if value_str[1] == 'b':
                msg.data = "s1"
                self.publisher_.publish(msg)
                msg.data = "v2"
                self.publisher_.publish(msg)
            elif value_str[1] == 'i':
                msg.data = "s2"
                self.publisher_.publish(msg)
                msg.data = "v2"
                self.publisher_.publish(msg)
            elif value_str[1] == 't':
                msg.data = "s3"
                self.publisher_.publish(msg)
                msg.data = "v2"
                self.publisher_.publish(msg)
            
            if value_str[2] == 'b':
                msg.data = "s1"
                self.publisher_.publish(msg)
                msg.data = "v3"
                self.publisher_.publish(msg)
            elif value_str[2] == 'i':
                msg.data = "s2"
                self.publisher_.publish(msg)
                msg.data = "v3"
                self.publisher_.publish(msg)
            elif value_str[2] == 't':
                msg.data = "s3"
                self.publisher_.publish(msg)
                msg.data = "v3"
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