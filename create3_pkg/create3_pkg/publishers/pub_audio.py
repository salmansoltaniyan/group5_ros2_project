#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote

class BeethovenPublisher(Node):
    def __init__(self):
        super().__init__("beethoven_publisher")
        
        self.publisher_ = self.create_publisher(AudioNoteVector, "/robot_9/cmd_audio", 10)
        
        # Wait 1 second to ensure the connection is established, then play!
        self.timer = self.create_timer(1.0, self.play_symphony)
        self.has_played = False

    def play_symphony(self):
        if self.has_played:
            return # We only want to play it once, not loop forever
            
        msg = AudioNoteVector()
        msg.append = False # Stop any current sound and play this immediately
        
        # Musical Note Frequencies (in Hz)
        C = 262
        D = 294
        E = 330
        F = 349
        G = 392
        
        # Beethoven's 9th (Ode to Joy) Melody
        # Tuple format: (Frequency, Duration in Seconds)
        melody = [
            (E, 0.4), (E, 0.4), (F, 0.4), (G, 0.4),  # E E F G
            (G, 0.4), (F, 0.4), (E, 0.4), (D, 0.4),  # G F E D
            (C, 0.4), (C, 0.4), (D, 0.4), (E, 0.4),  # C C D E
            (E, 0.6), (D, 0.2), (D, 0.8)             # E(long) D(short) D(long)
        ]
        
        # Build the message array
        for freq, duration in melody:
            note = AudioNote()
            note.frequency = freq
            
            # Convert decimal seconds (like 0.4) to ROS sec and nanosec format
            note.max_runtime.sec = int(duration)
            note.max_runtime.nanosec = int((duration - int(duration)) * 1_000_000_000)
            
            msg.notes.append(note)
            
        # Publish the song to the robot!
        self.publisher_.publish(msg)
        self.get_logger().info("🎵 Playing Beethoven's 9th Symphony (Ode to Joy) 🎵")
        
        self.has_played = True

def main(args=None):
    rclpy.init(args=args)
    node = BeethovenPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
