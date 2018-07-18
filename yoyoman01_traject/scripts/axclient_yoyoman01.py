
import roslib.message

from optparse import OptionParser
import wx
import rospy
import actionlib
import threading
import rostopic
from cStringIO import StringIO
from library import to_yaml, yaml_msg_str
from dynamic_action import DynamicAction
from actionlib_msgs.msg import GoalStatus




def main():
    rospy.init_node('axclient', anonymous=True)


#    parser.add_option("-t","--test",action="store_true", dest="test",default=False,
#                      help="A testing flag")
#  parser.add_option("-v","--var",action="store",type="string", dest="var",default="blah")



    if (len(args) == 2):
        # get action type via rostopic
        topic_type = rostopic._get_topic_type("%s/goal" % args[1])[0]
        # remove "Goal" string from action type
        assert("Goal" in topic_type)
        topic_type = topic_type[0:len(topic_type)-4]
    elif (len(args) == 3):
        topic_type = args[2]
        print(topic_type)
        assert("Action" in topic_type)


    action = DynamicAction(topic_type)
    app = AXClientApp(action, args[1])
    app.MainLoop()
    app.OnQuit()
    rospy.signal_shutdown('GUI shutdown')


if __name__ == '__main__':
    main()
