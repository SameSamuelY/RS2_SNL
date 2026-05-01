#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.command_publisher = self.create_publisher(String, '/gui_command', 10)

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.command_map = {
            "start": "start",
            "begin": "start",
            "go": "start",

            "stop": "stop",
            "emergency stop": "stop",
            "halt": "stop",

            "reset": "reset",
            "restart": "reset",

            "move": "move",
            "move robot": "move",

            "home": "home",
            "return home": "home",
        }

        self.get_logger().info("Voice command node started.")
        self.get_logger().info("Supported commands: start, stop, reset, move, home")

        with self.microphone as source:
            self.get_logger().info("Calibrating microphone noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        self.timer = self.create_timer(0.5, self.listen_for_command)

    def listen_for_command(self):
        try:
            with self.microphone as source:
                self.get_logger().info("Listening...")
                audio = self.recognizer.listen(
                    source,
                    timeout=3,
                    phrase_time_limit=3
                )

            recognised_text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Heard: {recognised_text}")

            for phrase, command in self.command_map.items():
                if phrase in recognised_text:
                    self.publish_command(command)
                    return

            self.get_logger().warn("No matching command found.")

        except sr.WaitTimeoutError:
            pass

        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio.")

        except sr.RequestError as error:
            self.get_logger().error(f"Speech recognition error: {error}")

        except Exception as error:
            self.get_logger().error(f"Voice node error: {error}")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

        self.get_logger().info(f"Published voice command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()