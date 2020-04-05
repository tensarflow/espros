#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16


class _GetchUnix:
    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def talker():

    pub_f = rospy.Publisher('/car/forward', Int16, queue_size=10)
    pub_b = rospy.Publisher('/car/backward', Int16, queue_size=10)
    pub_l = rospy.Publisher('/car/left', Int16, queue_size=10)
    pub_r = rospy.Publisher('/car/right', Int16, queue_size=10)

    rospy.init_node('car_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    getchar = _GetchUnix()

    while not rospy.is_shutdown():
        print("Give direction...")
        input_key = getchar.__call__()

        if input_key == 'w':
            print("'w' pressed")
            pub_f.publish(800)

        elif input_key == 's':
            print("'s' pressed")
            pub_b.publish(800)

        elif input_key == 'a':
            print("'a' pressed")
            pub_l.publish(600)

        elif input_key == 'd':
            print("'d' pressed")
            pub_r.publish(600)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
