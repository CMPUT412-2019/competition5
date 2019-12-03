import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def white_filter_orig(image):
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 70, 255])).astype(bool)


def white_filter(image):
    import cv2
    import numpy as np

    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    return cv2.inRange(hls, np.array([0, 230, 0]), np.array([180, 255, 255])).astype(bool)


def on_image(image, im_pub, cv_bridge):
    image = cv_bridge.imgmsg_to_cv2(image)
    white_mask = (white_filter(image) * 255).astype('uint8')
    im_pub.publish(cv_bridge.cv2_to_imgmsg(white_mask))


if __name__ == '__main__':
    rospy.init_node('script_test')
    cv_bridge = CvBridge()
    im_pub = rospy.Publisher('/viz/white_mask', Image)
    im_sub = rospy.Subscriber('/bottom_camera/image_raw', Image, lambda image: on_image(image, im_pub, cv_bridge))

    rospy.spin()
