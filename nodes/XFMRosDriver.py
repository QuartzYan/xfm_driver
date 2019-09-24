#!/usr/bin/env python
import rospy
import time
import threading
from XFMI2CDriver import XFMI2CDriver
from xfm_driver.msg import xfm_cmd
from xfm_driver.msg import xfm_status
from xfm_driver.msg import gpio_msg

class XFMRosDriver:
  def __init__(self):
    self._xfm = XFMI2CDriver()
    rospy.loginfo('get xfm firmware info')
    self.xfm_fm_version, self.xfm_build_num = self._xfm.getFWVersion()
    i = 1
    while not (self.xfm_fm_version != -1 and self.xfm_build_num != -1):
      self.xfm_fm_version, self.xfm_build_num = self._xfm.getFWVersion()
      if i % 10 == 0:
        rospy.logwarn("get xfm firmware error")
        exit(-1)
      i += 1
      time.sleep(1)
    rospy.loginfo('get xfm firmware successful!')

    self.status_pub = rospy.Publisher('xfm_status', xfm_status, queue_size=1)
    rospy.Subscriber('xfm_cmd', xfm_cmd, self.cmdCallBack)
    rospy.Subscriber('gpio_msg', gpio_msg, self.GPIOCallBack)

  def cmdCallBack(self, msg):
    if msg.xfm_reset == True:
      self._xfm.setReset()
    if msg.xfm_wakeup_disable == True:
      self._xfm.setWakeUpEn(0)
    else:
      self._xfm.setWakeUpEn(1)

  def GPIOCallBack(self, msg):
    if msg.xfm_wakeup_singal == 1:
      self.pushMsg()

  def pushMsg(self):
    msg = xfm_status()
    msg.header.frame_id = 'xfm_micro_link'
    msg.header.stamp = rospy.Time.now()
    msg.xfm_fm_version = self.xfm_fm_version
    msg.xfm_build_num = self.xfm_build_num
    msg.xfm_wakeup_angle = self._xfm.getWakeUpAng()
    i = 1
    while not (msg.xfm_wakeup_angle != -1):
      msg.xfm_wakeup_angle = self._xfm.getWakeUpAng()
      if i % 4 == 0:
        rospy.logfatal("get xfm wake-up angle error")
        msg.xfm_wakeup_angle = -1
        break
      i += 1
      time.sleep(0.5)

    msg.xfm_wakeup_score = self._xfm.getWakeUpScore()
    i = 1
    while not (msg.xfm_wakeup_score != -1):
      msg.xfm_wakeup_score = self._xfm.getWakeUpScore()
      if i % 4 == 0:
        rospy.logfatal("get xfm wake-up angle error")
        msg.xfm_wakeup_score = -1
        break
      i += 1
      time.sleep(0.5)

    self.status_pub.publish(msg)

def main():
  rospy.init_node('XFMRosDriver')
  xfm = XFMRosDriver()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
