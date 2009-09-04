#!/usr/bin/env python
import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy
from std_msgs.msg import Float32
import random
def gen_number():
    pub = rospy.Publisher('random_number', Float32)
    rospy.init_node('random_number_generator', log_level=roslib.msg.Log.INFO)
    rospy.loginfo("Generating random numbers")

    while not rospy.is_shutdown():
        pub.publish(Float32(random.normalvariate(5, 1)))
        rospy.sleep(0.05)

if __name__ == '__main__':
  try:
    gen_number()
  except Exception, e:
    print "Done"


    
  
