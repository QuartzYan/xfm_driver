#!/usr/bin/env python
import os
import sys
import time
import threading
from smbus2 import SMBus

XFM_I2C_BUS_NUM = 0
XFM_I2C_ADDRESS = 0x47
DELAY_TIME = 0.1 # 0.1S = 100ms

class XFMI2CDriver:
  def __init__(self):
    self.isOpen = False
    self._lock_ = threading.Lock()
    try:
      self.bus = SMBus(XFM_I2C_BUS_NUM)     
    except:
      print "open i2c bus %d failed!" % (XFM_I2C_BUS_NUM)
      exit(-1)
    else:
      self.isOpen = True
      print "open i2c bus %d succeed!" % (XFM_I2C_BUS_NUM)

  def getFWVersion(self):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x0f, 0x00, 0x00]
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        cer_vel = return_msg[2] | (return_msg[3] << 8)
        if return_msg[0] == 0x01 and cer_vel == 0x0002:
          time.sleep(0.001)
          return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x01, 4)
          self.XFMVersion = str(return_msg[1]) + "." + str(return_msg[0])
          time.sleep(0.001)
          return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x02, 4)
          self.XFMBuildNumInf = str(return_msg[0]) + "." + str(return_msg[1]) + "." + str(return_msg[2]) + "." + str(return_msg[3])
          return self.XFMVersion, self.XFMBuildNumInf
        else:
          print "get XFM firmware failed, please try again!"
          return -1, -1
    else:
      print "i2c bus is not open!"
      return -1, -1

  def getWakeUpAng(self):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x10, 0x00, 0x00]
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        cer_vel = return_msg[2] | (return_msg[3] << 8)
        if return_msg[0] == 0x01 and cer_vel == 0x0001:
          time.sleep(0.001)
          return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x01, 4)
          wakeup_ang = return_msg[0] | (return_msg[1] << 8)
          return wakeup_ang
        else:
          print "get XFM Wake-up angle failed, please try again!"
          return -1
    else:
      print "i2c bus is not open!"
      return -1

  def getWakeUpScore(self):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x16, 0x00, 0x00]
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        cer_vel = return_msg[2] | (return_msg[3] << 8)
        if return_msg[0] == 0x01 and cer_vel == 0x0001:
          time.sleep(0.001)
          return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x01, 4)
          wakeup_score = return_msg[0] | (return_msg[1] << 8)
          return wakeup_score
        else:
          print "get XFM Wake-up score failed, please try again!"
          return -1
    else:
      print "i2c bus is not open!"
      return -1

  # def getKeyWordID(self):
  #   if self.isOpen:
  #     inquire_msg = [0x00, 0x15, 0x00, 0x00]
  #     self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
  #     time.sleep(DELAY_TIME)
  #     return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
  #     cer_vel = return_msg[2] | (return_msg[3] << 8)
  #     if return_msg[0] == 0x01:
  #       return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x01, 4)
  #       keyword_id = return_msg[0] | (return_msg[1] << 8)
  #       return wakeup_score
  #     else:
  #       print "get XFM key-word id error, please try again!"
  #       return -1
  #   else:
  #     print "i2c bus is not open!"
  #     return -1

  def setReset(self):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x11, 0x00, 0x00]
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        if return_msg[0] == 0x01:
          return True
        else:
          print "set XFM reset failed, please try again!"
          return False
    else:
      print "i2c bus is not open!"
      return -1

  def setListenAng(self, listen_ang):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x12, 0x00, 0x00]
        default_listen_ang = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]
        if listen_ang in default_listen_ang:
          inquire_msg[2] = listen_ang
        else:
          print "input listen-angle:%s error, 0x00--0x05" % (str(listen_ang))
          return False
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        cer_vel = return_msg[2] | (return_msg[3] << 8)
        if return_msg[0] == 0x01 and cer_vel == 0x0003:
          return True
        else:
          print "set XFM listen-angle failed, please try again!"
          return False
    else:
      print "i2c bus is not open!"
      return -1

  def setWakeUpEn(self, wakeup_en):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x13, 0x01, 0x00]
        if wakeup_en == 0:
          inquire_msg[2] = 0x00
        elif wakeup_en == 1:
          inquire_msg[2] = 0x01
        else:
          print "input %s error, 0:disable wake-up, 1:enable wake-up" % (str(wakeup_en))
          return False
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        if return_msg[0] == 0x01:
          return True
        else:
          print "set XFM wake-up en failed, please try again!"
          return False
    else:
      print "i2c bus is not open!"
      return -1

  def setCallMode(self, mode):
    if self.isOpen:
      with self._lock_:
        inquire_msg = [0x00, 0x14, 0x01, 0x00]
        if mode == 0:
          inquire_msg[2] = 0x00
        elif mode == 1:
          inquire_msg[2] = 0x01
        else:
          print "input mode:%s error, 0:single-mc operating mode, 1:sound source positioning function" % (str(mode))
          return False
        self.bus.write_i2c_block_data(XFM_I2C_ADDRESS, 0x00, inquire_msg)
        time.sleep(DELAY_TIME)
        return_msg = self.bus.read_i2c_block_data(XFM_I2C_ADDRESS, 0x00, 4)
        if return_msg[0] == 0x01:
          return True
        else:
          print "set XFM Call Mode failed, please try again!"
          return False
    else:
      print "i2c bus is not open!"
      return -1
