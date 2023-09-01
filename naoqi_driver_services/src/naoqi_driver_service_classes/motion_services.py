import rospy
from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledResponse
from nao_interaction_msgs.srv import GoToPose, GoToPoseResponse
from nao_interaction_msgs.srv import MotionSetAngles, MotionSetAnglesResponse
from nao_interaction_msgs.srv import MotionInterpolate, MotionInterpolateResponse
from std_srvs.srv import Empty, EmptyResponse
import utils as ut
import tf


class MotionServices(AbstractService):
    def __init__(self, super_ns):
        super(MotionServices, self).__init__(
            proxy_name="ALMotion",
            ns=super_ns+"/motion",
            topics=["move_to", "rest", "set_breath_enabled", "wake_up", "set_angles", "angle_interpolation", "angle_interpolation_with_speed", "neutral"],
            service_types=[GoToPose, Empty, SetBreathEnabled, Empty, MotionSetAngles, MotionInterpolate, MotionSetAngles, Empty])
        self.listener = tf.TransformListener()

    def transform(self, pose_stamped, target_frame):
        if pose_stamped.header.frame_id != target_frame:
            try:
                t = self.listener.getLatestCommonTime(target_frame, pose_stamped.header.frame_id)
                pose_stamped.header.stamp = t
                return self.listener.transformPose(target_frame, pose_stamped)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                return None
        else:
            return pose_stamped

    def move_to_callback(self, req):
        req.pose = self.transform(req.pose, "base_footprint")

        yaw = ut.get_yaw(req.pose.pose)

        rospy.loginfo("going to move x: {x} y: {y} z: {z} yaw: {yaw}".format(
            x=req.pose.pose.position.x,
            y=req.pose.pose.position.y,
            z=req.pose.pose.position.z,
            yaw=yaw))
        self.proxy.moveTo(req.pose.pose.position.x, req.pose.pose.position.y, yaw)

        return GoToPoseResponse()

    def rest_callback(self, req):
        self.proxy.rest()
        return EmptyResponse()

    def set_breath_enabled_callback(self, req):
        if req.chain_name in (req.HEAD, req.BODY, req.ARMS, req.LEGS, req.LARM, req.RARM):
            self.proxy.setBreathEnabled(req.chain_name, req.enable)
        else:
            rospy.logerr("Unknown body part")
        return SetBreathEnabledResponse()

    def wake_up_callback(self, req):
        self.proxy.wakeUp()
        return EmptyResponse()

    def set_angles_callback(self, req):
        if len(req.names) != len(req.angles):
            rospy.logerr("Size of names different from size of angles")
        else:
            self.proxy.setAngles(req.names, req.angles, req.max_speed_fraction)
        return MotionSetAnglesResponse()

    def angle_interpolation_with_speed_callback(self, req):
        if len(req.names) != len(req.angles):
            rospy.logerr("Size of names different from size of angles")
        else:
            self.proxy.angleInterpolationWithSpeed(req.names, req.angles, req.max_speed_fraction)
        return MotionSetAnglesResponse()

    def angle_interpolation_callback(self, req):
        #if len(req.names) != len(req.angles):
        #    rospy.logerr("Size of names different from size of angles")
        #else:
        angles = []
        for angle in req.angles:
            angles.append(angle.values)
        times = []
        for t in req.times:
            times.append(t.values)

        print(req.names, angles, times)

        self.proxy.angleInterpolation(req.names, angles, times, req.is_absolute)
        return MotionInterpolateResponse()

    def neutral_callback(self, _):
        self.proxy.setAngles(["Body"], [0.0193678308,-0.204438269,1.24219799,0.0365650989,-1.11843324,-0.526670039,-1.03511024,0.61011523,-2.51969592e-08,-0.0573792122,-0.0442125686,1.31760716,-0.0351500735,1.37310743,0.613771081,0.806987405,0.688877523,0,0,0], 0.1)
        return EmptyResponse()

