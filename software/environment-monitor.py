#!/usr/bin/env python
"""environment-monitor rosnode

https://github.com/strawlab/environment-monitor

This will publish to:
/environment-monitor/tc_temperature_C
/environment-monitor/hs_temperature_C
/environment-monitor/hs_humidity
/environment-monitor/ls_infrared
/environment-monitor/ls_visible
/environment-monitor/ls_full
/environment-monitor/ls_intensity_lux

The subtopicnames are also defined in the firmware
"""
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('std_msgs')
import rospy
import std_msgs.msg
import re
import serial


class EnvironmentMonitor(object):

    def __init__(self, dev, stdout=False):
        self._stdout = stdout
        self._dev = serial.Serial(dev, baudrate=115200)
        self._buffer = ""

        self.topicnames = ["tc_temperature_C",
                           "hs_temperature_C",
                           "hs_humidity",
                           "ls_infrared",
                           "ls_visible",
                           "ls_full",
                           "ls_intensity_lux"]
        self.pubs = map(
            lambda x: rospy.Publisher("/environmentmonitor/%s" % x, std_msgs.msg.Float64),
            self.topicnames)

    def run(self, *rostimer_args):
        input_N = self._dev.inWaiting()
        self._buffer += self._dev.read(input_N)
        while True:
            try:
                idx = self._buffer.index('\n')
            except ValueError:
                break
            else:
                line = self._buffer[:idx]
                self.publish(line)
                self._buffer = self._buffer[idx+1:]
        return

    def publish(self, string):
        for tn, pub in zip(self.topicnames, self.pubs):
            m = re.search("%s=([0-9.]+)" % tn, string)
            if m is not None:
                pub.publish(float(m.group(1)))
                if self._stdout:
                    print tn, float(m.group(1))

if __name__ == '__main__':
    import argparse


    rospy.init_node('environmentmonitor')

    em = EnvironmentMonitor('/dev/ttyUSB0', stdout=True)
    rospy.Timer(rospy.rostime.Duration.from_sec(0.5), em.run)

    rospy.spin()
