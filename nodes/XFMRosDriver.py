#!/usr/bin/env python
import rospy
import time
import gpio
import threading

from XFMI2CDriver import XFMI2CDriver
from xfm_driver.msg import xfm_cmd
from xfm_driver.msg import xfm_status

XFM_WAKEUP_PIN = 480

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

    rospy.loginfo('GPIO setup')
    try:
      gpio.setup(XFM_WAKEUP_PIN, gpio.IN)
      rospy.loginfo('GPIO setup successful!!')
    except:
      rospy.loginfo('GPIO setup try again...')
      time.sleep(2)
      try:
        gpio.setup(XFM_WAKEUP_PIN, gpio.IN)
        rospy.loginfo('GPIO setup successful!!')
      except:
        rospy.loginfo('GPIO setup failed!!')
        exit(-1)      

    self.status_pub = rospy.Publisher('xfm_status', xfm_status, queue_size=1)
    rospy.Subscriber('xfm_cmd', xfm_cmd, self.cmdCallBack)

  def cmdCallBack(self, msg):
    if msg.xfm_reset == True:
      self._xfm.setReset()
    if msg.xfm_wakeup_disable == True:
      self._xfm.setWakeUpEn(0)
    elif msg.xfm_reset == False:
      self._xfm.setWakeUpEn(1)

  def getGPIO(self):
    rospy.loginfo('start scan gpio...')
    xfm_last_status = gpio.read(XFM_WAKEUP_PIN)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
      xfm_now_status = gpio.read(XFM_WAKEUP_PIN)
      if xfm_last_status == 0 and xfm_now_status == 1:
        #print "RISING!!"
        self.pushMsg()
      xfm_last_status = xfm_now_status
      r.sleep()
      
  def getMsgWithoutGPIO(self):
    rospy.loginfo('get i2c massage without gpio...')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.pushMsg() 
      r.sleep()

  def pushMsg(self):
    msg = xfm_status()
    msg.header.frame_id = 'xfm_micro_link'
    msg.header.stamp = rospy.Time.now()
    msg.xfm_fm_version = self.xfm_fm_version
    msg.xfm_build_num = self.xfm_build_num

    wakeup_ang = self._xfm.getWakeUpAng()
    i = 1
    while not (wakeup_ang != -1):
      wakeup_ang = self._xfm.getWakeUpAng()
      if i % 4 == 0:
        rospy.logfatal("get xfm wake-up angle error")
        wakeup_ang = -111
        break
      i += 1
      time.sleep(0.5)
    msg.xfm_wakeup_angle = self.ang2rad(wakeup_ang)

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

  def ang2rad(self, ang):
    if ang >= 0 and ang < 180:
      return -(180-ang)*0.01745329
    elif ang >= 180 and ang < 360:
      return (ang-180)*0.01745329
    else:
      return 0
      print "input error"

def main():
  rospy.init_node('XFMRosDriver')
  xfm = XFMRosDriver()
  # th = threading.Thread(target=xfm.getGPIO)
  th = threading.Thread(target=xfm.getMsgWithoutGPIO)
  th.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
