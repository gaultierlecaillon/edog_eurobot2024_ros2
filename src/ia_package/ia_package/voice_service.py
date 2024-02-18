#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from example_interfaces.msg import String
from playsound import playsound

class VoiceService(Node):   

    def __init__(self):
        super().__init__("voice_service")

        self.last_time_spoke = self.get_clock().now().to_msg().sec
        self.previous_audio = ""

        ''' Subscribers '''
        self.create_subscription(
            String,
            'voice_topic',
            self.voice_callback,
            10)
        

        self.get_logger().info('VoiceService node has started')


    def play_mp3(self, file_path):
        play_sound = False
        current_time = self.get_clock().now().to_msg().sec
        
        if self.previous_audio  == "emergency_stop.mp3" and current_time - self.last_time_spoke > 3: #avoid spam on emmergency
            play_sound = True
        elif current_time - self.last_time_spoke > 0.1:
            play_sound = True

        if play_sound:
            try:
                playsound(file_path)
                # Update last_time_spoke to current time after speaking
                self.last_time_spoke = current_time
                self.previous_audio = file_path
            except Exception as e:
                self.get_logger().info(f"\033[91mError occurred while playing sound '{file_path}': {e}\033[0m")
        else:
            self.get_logger().info("\033[93mWait for 0.5 seconds since the last speech.\033[0m")

            



    def voice_callback(self, msg):
        self.get_logger().info(f"\033[94m[VOICE] Received: {msg.data}\033[0m")
        mp3_file = '/home/edog/ros2_ws/src/ia_package/resource/audio_files/' + str(msg.data)    
        self.play_mp3(mp3_file)



def main(args=None):
    rclpy.init(args=args)
    node = VoiceService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
