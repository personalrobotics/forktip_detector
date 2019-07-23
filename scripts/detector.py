#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('forktip_detector')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as pyplot
import numpy

# FORK_U_OFFSET = 36
# FORK_V_OFFSET = 17

FORK_U_OFFSET = 39
FORK_V_OFFSET = 17

class tip_detector:

  def __init__(self):
    uv_topic = rospy.get_param("fork_uv_topic", "fork_uv")
    self.uv_pub = rospy.Publisher(uv_topic, Pose2D, queue_size=1)

    self.bridge = CvBridge()
    image_topic = rospy.get_param("image_topic", "/camera/color/image_raw/")
    self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)

    self.template = cv2.imread('template_mid.png', 0)

  def callback(self, data):
    # Convert to cv2 color image
    try:
      img = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    # Pre-Process Image
    y = 150
    x = 300
    img = img[y:y+280, x:x+250]
    img = cv2.convertScaleAbs(img, alpha=1.7, beta=-1.4)
    blur = cv2.GaussianBlur(img, (7, 7), 0)
    blur_edges = cv2.Canny(blur, 0, 140)

    # Template Matching
    res = cv2.matchTemplate(blur_edges, self.template, cv2.TM_CCORR)
    cv2.normalize(res, res, 0, 1, cv2.NORM_MINMAX, -1);

    _, max_val, _, max_loc = cv2.minMaxLoc(res)

    u = max_loc[0] + FORK_U_OFFSET
    v = max_loc[1] + FORK_V_OFFSET

    print("Found fork at: (%d, %d)" % (u + x, v + y))
    print("Strength: " + str(max_val))

    # Publish Pose Message
    pose = Pose2D()
    pose.x = u + x
    pose.y = v + y
    pose.theta = max_val
    self.uv_pub.publish(pose)


def main(args):
  td = tip_detector()
  rospy.init_node('forktip_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)