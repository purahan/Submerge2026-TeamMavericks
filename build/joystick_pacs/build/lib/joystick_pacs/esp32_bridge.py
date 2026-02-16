import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
import serial
import struct

class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.subscription = self.create_subscription(
            RovCommands, '/nemo_auv/input_cmd', self.listener_callback, 10)
        
        # Adjust port to your Pi's connection (usually /dev/ttyUSB0 or /dev/ttyACM0)
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial Connection Failed: {e}")

    def listener_callback(self, msg):
	    # For uni-directional, we assume you only want positive thrust 
	    # (or you have motors facing opposite directions for forward/back)
	    
	    # Simple example: Surge only moves you forward
	    # We use max(0, val) to ensure we never send a "negative" (reverse) command
        m_fr = max(0.0, msg.surge + msg.yaw)
        m_fl = max(0.0, msg.surge - msg.yaw)
        m_vr = max(0.0, msg.heave) # Vertical motors
        m_vl = max(0.0, msg.heave)

        outputs = []
        for val in [m_fr, m_fl, m_vr, m_vl]:
            val = min(1.0, val) # Clamp at 100%
            byte_val = int(val * 255) # 0.0 is stop, 1.0 is full speed
            outputs.append(byte_val)

        packet = bytearray([0xAA, outputs[0], outputs[1], outputs[2], outputs[3], 0x55])
        self.ser.write(packet)

def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
