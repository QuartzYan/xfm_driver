#!/usr/bin/env python
import time
import gpio
import rospy
from xfm_driver.msg import gpio_msg

XFM_WAKEUP_PIN = 480
WIFI_SWITCH_PIN = 298

def main():
  rospy.init_node('get_gpio')
  gpio_pub = rospy.Publisher('gpio_msg', gpio_msg, queue_size=1)
  
  gpio.setup(XFM_WAKEUP_PIN, gpio.IN)
  gpio.setup(WIFI_SWITCH_PIN, gpio.IN)

  xfm_last_status = gpio.read(XFM_WAKEUP_PIN)
  wifi_last_status = gpio.read(WIFI_SWITCH_PIN)
  
  rospy.loginfo('start scan gpio...')

  r = rospy.Rate(100)
  while not rospy.is_shutdown():
    xfm_now_status = gpio.read(XFM_WAKEUP_PIN)
    wifi_now_status = gpio.read(WIFI_SWITCH_PIN)
    if xfm_last_status==0 and xfm_now_status==1:
      #print "RISING!!"
      msg = gpio_msg()
      msg.xfm_wakeup_singal = 1
      msg.wifi_switch_singal = wifi_now_status
      gpio_pub.publish(msg)
    #elif xfm_last_status==1 and xfm_now_status==0:
      #pass
      #print "FALLING!!"

#    if wifi_last_status==0 and wifi_now_status==1:
#      msg = gpio_msg()
#      msg.xfm_wakeup_singal = 0
#      msg.wifi_switch_singal = wifi_now_status
#      gpio_pub.publish(msg)
#    elif wifi_last_status==1 and wifi_now_status==0:
#      msg = gpio_msg()
#      msg.xfm_wakeup_singal = 0
#      msg.wifi_switch_singal = wifi_now_status
#      gpio_pub.publish(msg)

    xfm_last_status = xfm_now_status
#    wifi_last_status = wifi_now_status

    r.sleep()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
