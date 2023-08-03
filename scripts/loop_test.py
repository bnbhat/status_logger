import time
import os
from status_logger import StatusLogger
import rospy
import rostest
from kopro_msgs.srv import kopro_scheduler_statusLogger_initial
from kopro_msgs.msg import kopro_scheduler_statusLogger_log

# Test
def test():
    logger = StatusLogger()

    rospy.wait_for_service('/kopro_scheduler_statusLogger_initial')
    init_service = rospy.ServiceProxy('/kopro_scheduler_statusLogger_initial', kopro_scheduler_statusLogger_initial)
    publisher = rospy.Publisher('/kopro_scheduler_statusLogger_log', kopro_scheduler_statusLogger_log, queue_size=10)

    # Start the log writer
    response = init_service(START, 'test_file')
    if response:
        rospy.loginfo("Logger started successfully")
    else:
        rospy.logerr("Logger failed to start")
        exit()
    
    # Give some time for the logger to start
    time.sleep(2)

    # Send a log message
    for i in range(10):
        msg = kopro_scheduler_statusLogger_log()
        msg.uuid = "test_uuid"
        msg.command = 3
        msg.resource_type = 1
        msg.msg = "Test message"
        msg.time = rospy.Time.now()

        publisher.publish(msg)

        # Give some time for the message to be logged
        time.sleep(2)
    
    # Stop the log writer
    response = init_service(STOP)
    if response:
        rospy.loginfo("Logger stopped successfully")
    else:
        rospy.logerr("Logger failed to stop")
        exit()

    # Give some time for the logger to stop
    time.sleep(2)

    rospy.spin()