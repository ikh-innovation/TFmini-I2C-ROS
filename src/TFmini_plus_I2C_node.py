#!/usr/bin/env python2

import rospy
from TFmini_plus_I2C import TFminiI2C
from sensor_msgs.msg import Range

class TFmini_ROS(TFminiI2C):
    def __init__(self, name, bus, id):
        self.I2Cbus = bus
        self.device_address = id
        self.name = name
        self.min_range = 0.1
        self.max_range = 12.0
        TFminiI2C.__init__(self, self.I2Cbus , self.device_address)

        # Init Publisher
        self.range_pub = rospy.Publisher(name , Range, queue_size=10)

        # Init Range message
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.INFRARED
        self.range_msg.header.frame_id = self.name
        self.range_msg.field_of_view = 0.4
        self.range_msg.min_range = self.min_range
        self.range_msg.max_range = self.max_range

    # TODO add strength signal to condition
    def fill_msg(self):
        _data = self.readAll()
        _dist = float(_data[1])/100
        _strength = _data[2] 
        self.range_msg.header.stamp = rospy.Time.now()

        if _dist > self.min_range and _dist < self.max_range:
            self.range_msg.range = _dist
        elif _dist == -1.0:
            rospy.logerr("Failed to read data. TFmini ros node stopped!")
            return

        
    def range_publish(self):
        try :
            self.fill_msg()
            self.range_pub.publish(self.range_msg)

        except OSError as error :
            pass
       

if __name__ == '__main__':
    try:
        rospy.init_node('TFmini_plus_I2C_node')
        rate = rospy.Rate(2)

        lidars = []
        bus = rospy.get_param('i2cbus')

        lidars_list = rospy.get_param('lidars')

        for lidar in lidars_list.keys():
            address = lidars_list[lidar]['device_address']
            lidars.append(TFmini_ROS(lidar, bus, address))

        while not rospy.is_shutdown():
            for i in range(0,len(lidars)):   
                lidars[i].range_publish()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass