#------------------------------------------------------------------------------------------------------------
# File: scripts/status_logger.py
# Description: This file contains the status logger. It logs the status msgs from the scheduler.
# Author: Balachandra Bhat (github.com/bnbhat)
# Date: 2023-08-03
# Version: 1.0
#------------------------------------------------------------------------------------------------------------

import time
import rospy
import log_writer

from kopro_msgs.msg import kopro_scheduler_statusLogger_log
from kopro_msgs.srv import kopro_scheduler_statusLogger_initial, \
                            kopro_scheduler_statusLogger_initialResponse,\
                            kopro_scheduler_statusLogger_initialRequest

#init commands
START = 1
STOP = 2

class StatusLogger:

    def __init__(self):
        self.log_writer = None
        self.init_ros()

    def init_ros(self):
        rospy.init_node('status_logger')
        rospy.loginfo("Status Logger node is starting")
        self.subscriber = rospy.Subscriber('/kopro_scheduler_statusLogger_log', kopro_scheduler_statusLogger_log, self.log_callback)
        self.initial_service = rospy.Service('/kopro_scheduler_statusLogger_initial', kopro_scheduler_statusLogger_initial, self.init_callback)
    
    def init_callback(self, req):
        if req.command == START:
            try:
                if self.log_writer is not None:
                    self.log_writer.close()
                else:
                    self.log_writer = log_writer.LogWriter(req.file_name)
                    rospy.loginfo("Status Logger is started")
                    return kopro_scheduler_statusLogger_initialResponse(1)
            except Exception as e:
                rospy.logerr(f"Error initiating log file: {e}")
                return kopro_scheduler_statusLogger_initialResponse(0)
            
        elif req.command == STOP:
            try:
                if self.log_writer is not None:
                    rospy.loginfo("Please initailize the logger before stopping it")
                    return kopro_scheduler_statusLogger_initialResponse(0)
                else:
                    self.log_writer.close()
                    rospy.loginfo("Status Logger is finished")
                    return kopro_scheduler_statusLogger_initialResponse(1) 
            except Exception as e:
                rospy.logerr(f"Error closing log file: {e}")
                return kopro_scheduler_statusLogger_initialResponse(0)
        
    def log_callback(self, msg):
        if self.log_writer is not None:
            try:
                self.log_writer.log(msg.uuid, msg.command, msg.resource_type, msg.msg, self.ros_time_to_HMS(msg.time))
            except Exception as e:
                rospy.logerr(f"Error logging to file: {e}")
        else:
            rospy.logerr("Please initailize the logger before logging")

    def ros_time_to_HMS(self, ros_time):
        if ros_time is None:
            return time.strftime("%H:%M:%S", time.localtime(time.time()))
        else:
            try:
                return time.strftime("%H:%M:%S", time.localtime(ros_time.to_sec()))
            except Exception as e:
                rospy.logerr(f"Error converting ros time to HMS: {e}")
                return time.strftime("%H:%M:%S", time.localtime(time.time()))

if __name__ == "__main__":
    logger = StatusLogger()
    rospy.spin()