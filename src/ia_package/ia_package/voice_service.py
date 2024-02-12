#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from example_interfaces.msg import String
import pygame

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


    def play_mp3(self, file_path, timeout=30):
        try:
            # Initialize the pygame mixer
            pygame.mixer.init()
            
            # Load the MP3 file
            pygame.mixer.music.load(file_path)
            
            # Play the MP3 file
            pygame.mixer.music.play()
            
            # Setup for timeout
            start_time = time.time()
            
            # Wait for the music to play with a timeout
            while pygame.mixer.music.get_busy():
                time.sleep(1)
                if (time.time() - start_time) > timeout:
                    self.get_logger().info(f"\033[91mError: Music play timeout exceeded.\033[0m")  # Error message in red
                    break
            
        except Exception as e:
            # Print the error message in red
            self.get_logger().info(f"\033[91mError occurred: {e}\033[0m")
            



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
