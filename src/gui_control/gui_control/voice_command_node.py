#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

import speech_recognition as sr


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.command_publisher = self.create_publisher(String, '/gui_command', 10)

        self.trigger_client = self.create_client(
            Trigger,
            '/trigger_pick_and_place'
        )

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.command_map = {
            "start": "start",
            "begin": "start",
            "go": "start",
            "run": "start",

            "move": "move",
            "move robot": "move",
            "pick and place": "move",
            "pick": "move",

            "stop": "stop",
            "emergency stop": "stop",
            "halt": "stop",
            "pause": "stop",

            "reset": "reset",
            "restart": "reset",

            "home": "home",
            "return home": "home",
        }

        self.get_logger().info("Voice command node started.")
        self.get_logger().info("Commands: start, stop, reset, move, home")

        try:
            with self.microphone as source:
                self.get_logger().info("Calibrating microphone noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        except Exception as error:
            self.get_logger().error(f"Microphone setup error: {error}")

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
                    self.handle_command(command)
                    return

            self.get_logger().warn("No matching voice command found.")

        except sr.WaitTimeoutError:
            pass

        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio.")

        except sr.RequestError as error:
            self.get_logger().error(f"Speech recognition API error: {error}")

        except Exception as error:
            self.get_logger().error(f"Voice node error: {error}")

    def handle_command(self, command):
        self.publish_gui_command(command)

        if command in ["start", "move"]:
            self.call_pick_and_place_trigger()

    def publish_gui_command(self, command):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

        self.get_logger().info(f"Published voice command to /gui_command: {command}")

    def call_pick_and_place_trigger(self):
        if not self.trigger_client.service_is_ready():
            if not self.trigger_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    "/trigger_pick_and_place service unavailable. "
                    "Make sure mtc_pick_place_listener is running."
                )
                return

        request = Trigger.Request()
        future = self.trigger_client.call_async(request)

        future.add_done_callback(self.trigger_response_callback)

        self.get_logger().info("Voice command called /trigger_pick_and_place service")

    def trigger_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Trigger response: success={response.success}, message={response.message}"
            )

        except Exception as error:
            self.get_logger().error(f"Trigger service call failed: {error}")


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