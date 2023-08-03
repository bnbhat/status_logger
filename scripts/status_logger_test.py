import unittest
import time
import os
import csv
import rospy
import rostest
from kopro_msgs.srv import kopro_scheduler_statusLogger_initial
from kopro_msgs.msg import kopro_scheduler_statusLogger_log

PKG = 'your_package'
NAME = 'status_logger_test'

START = 1
STOP = 0

class StatusLoggerTest(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/kopro_scheduler_statusLogger_initial')
        self.init_service = rospy.ServiceProxy('/kopro_scheduler_statusLogger_initial', kopro_scheduler_statusLogger_initial)
        self.publisher = rospy.Publisher('/kopro_scheduler_statusLogger_log', kopro_scheduler_statusLogger_log, queue_size=10)

    def test_log_writer(self):
        # Start the log writer
        response = self.init_service(START, 'test_file')
        self.assertTrue(response.result, "Failed to start the logger")

        # Give some time for the logger to start
        time.sleep(2)

        # Send a log message
        msg = kopro_scheduler_statusLogger_log()
        msg.uuid = "test_uuid"
        msg.command = 3
        msg.resource_type = 1
        msg.msg = "Test message"
        msg.time = rospy.Time.now()

        self.publisher.publish(msg)

        # Give some time for the message to be logged
        time.sleep(2)

        # Stop the log writer
        response = self.init_service(STOP)
        self.assertTrue(response.result, "Failed to stop the logger")

        # Give some time for the logger to stop
        time.sleep(2)

        # Check the log file
        base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
        file_path = os.path.join(base_dir, 'logs', 'test_file.txt')
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            lines = list(reader)

        self.assertEqual(len(lines), 1, "Wrong number of log entries")
        self.assertEqual(lines[0][0], "test_uuid", "Wrong UUID")
        self.assertEqual(int(lines[0][1]), 3, "Wrong command")
        self.assertEqual(int(lines[0][2]), 1, "Wrong resource type")
        self.assertEqual(lines[0][3], "Test message", "Wrong message")

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, StatusLoggerTest)
