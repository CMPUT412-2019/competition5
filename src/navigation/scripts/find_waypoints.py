import rospy
from sensor_msgs.msg import Joy
from tf.listener import TransformListener
from os import path

from src.util.scripts.util import SubscriberValue


def main():
    rospy.init_node('find_waypoints')
    joy_subscriber = SubscriberValue('joy', Joy)
    tf_listener = TransformListener()

    names = ['off_ramp_start', 'off_ramp_end', 'ar_search_1', 'ar_search_2', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8', 'on_ramp']
    coords = []

    for name in names:
        rospy.sleep(rospy.Duration(2))
        print(name)
        while not joy_subscriber.value.buttons[0]:
            rospy.sleep(rospy.Duration(0, 1000))

        pose = tf_listener.lookupTransform('/map', '/base_link', rospy.Time())
        coords.append(pose)
        print('Saved')

    out_file = open(path.join(path.dirname(__file__), '..', 'param', 'find_waypoints_generated.yaml'), 'w')
    out_file.write('named_poses:\n')
    for name, ((x, y, z), (rx, ry, rz, rw)) in zip(names, coords):
        out_file.write('  {name}:\n'.format(name=name))
        out_file.write('    position:\n')
        out_file.write('      - {x}\n'.format(x=x))
        out_file.write('      - {y}\n'.format(y=y))
        out_file.write('      - {z}\n'.format(z=z))
        out_file.write('    orientation:\n')
        out_file.write('      - {x}\n'.format(x=rx))
        out_file.write('      - {y}\n'.format(y=ry))
        out_file.write('      - {z}\n'.format(z=rz))
        out_file.write('      - {w}\n'.format(w=rw))


if __name__ == '__main__':
    main()
