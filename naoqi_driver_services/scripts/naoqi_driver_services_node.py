#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import qi
import sys

from naoqi_driver_service_classes.tts_services import TTSServices
from naoqi_driver_service_classes.animated_speech_services import AnimatedSpeechServices
from naoqi_driver_service_classes.motion_services import MotionServices
from naoqi_driver_service_classes.behaviour_manager_services import BehaviourManagerServices
from naoqi_driver_service_classes.tracker_services import TrackerServices
from naoqi_driver_service_classes.tablet_services import TabletServices
from naoqi_driver_service_classes.led_services import LedServices


class ServicesNode(object):
    def __init__(self, name, session):
        rospy.loginfo("Starting %s ..." % name)
        super_ns = rospy.get_param("~super_ns", "/naoqi_driver")
        TTSServices(session, super_ns)
        AnimatedSpeechServices(session, super_ns)
        MotionServices(session, super_ns)
        BehaviourManagerServices(session, super_ns)
        TrackerServices(session, super_ns)
        TabletServices(session, super_ns)
        LedServices(session, super_ns)
        rospy.loginfo("... done")

if __name__ == "__main__":
    rospy.init_node("naoqi_driver_service_node")
    session = qi.Session()
    NAO_IP = rospy.get_param("~nao_ip", "pepper")
    PORT = rospy.get_param("~nao_port", 9559)

    try:
        session.connect(f"tcp://{NAO_IP}:{PORT}")
    except RuntimeError:
        print(f"Can't connect to Naoqi at {NAO_IP}:{PORT}.")
        sys.exit(1)

    s = ServicesNode(rospy.get_name(), session)
    rospy.spin()

    session.close()
