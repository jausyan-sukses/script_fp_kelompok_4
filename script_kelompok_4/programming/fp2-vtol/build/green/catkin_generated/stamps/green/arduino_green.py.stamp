import rospy
from std_msgs.msg import String
import serial

class ArduinoCommunicator:
    def __init__(self, port, baud_rate):

        self.serial = serial.Serial(port, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to Arduino on {port} at {baud_rate} baud.")

        rospy.init_node('arduino_communicator_node', anonymous=True)

        rospy.Subscriber('movement_commands', String, self.callback)

    def callback(self, data):
        command = data.data
        rospy.loginfo(f"Received command: {command}")
        self.send_to_arduino(command)

    def send_to_arduino(self, command):
        self.serial.write((command + '\n').encode())
        rospy.loginfo(f"Sent to Arduino: {command}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        communicator = ArduinoCommunicator('/dev/ttyUSB0', 9600)  
        communicator.spin()
    except rospy.ROSInterruptException:
        pass