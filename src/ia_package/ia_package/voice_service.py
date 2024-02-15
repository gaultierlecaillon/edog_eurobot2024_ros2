#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from example_interfaces.msg import String
from playsound import playsound

class VoiceService(Node):   

    def __init__(self):
        super().__init__("voice_service")

        ''' Subscribers '''
        self.create_subscription(
            String,
            'voice_topic',
            self.voice_callback,
            10)
        

        self.get_logger().info('VoiceService node has started')


    def play_mp3(self, file_path):
        try:
            playsound(file_path)
        except Exception as e:
            # Print the error message in red
            self.get_logger().info(f"\033[91mError occurred while playing song '{file_path}': {e}\033[0m")
            



    def voice_callback(self, msg):
        self.get_logger().info(f"\033[94m[VOICE] Received: {msg.data}\033[0m")
        mp3_file = '/home/edog/Robot/Bluetooth/soundtrack/' + str(msg.data)    
        self.play_mp3(mp3_file)



def main(args=None):
    rclpy.init(args=args)
    node = VoiceService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
