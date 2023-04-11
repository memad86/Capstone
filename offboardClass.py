#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import Bool 
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandTOL, CommandBoolRequest

class OFFBOARD:
    def __init__(self, id):
        self.uav_ID = id
        rospy.init_node("uav_" + str(self.uav_ID) + "_offb_node", anonymous=False)
        self.rate                               = rospy.Rate(300)
        self.lastTime                           = rospy.Time.now()
        self.current_state                      = State() 
        self.targetVelocity                     = PositionTarget()
        self.height                             = Point()
        self.takeOffMode                        = False
        self.landingMode                        = False
        self.offboardMode                       = False
        self.velocityMode                       = False
        # ------------------------------------- #
        # ************PUBLISHERS*************** #
        # ------------------------------------- #
        self.localPositionPublisher             = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.localVelocityPublisher             = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # ------------------------------------- #
        # ************SUBSCRIBERS************** #
        # ------------------------------------- #
        self.stateSubscriber                    = rospy.Subscriber("/mavros/state", State, self.stateCallback)
        self.targetVelocitySubscriber           = rospy.Subscriber("/target_velocity", PositionTarget, self.targetVelocityCallback)
        self.heightSubsciber                    = rospy.Subscriber("/height", Point, self.heightCallback)
                    # MODES
        self.takeOffModeSubscriber              = rospy.Subscriber("/takeOff", Bool, self.takeOffModeCallback)
        self.landingModeSubscriber              = rospy.Subscriber("/landing", Bool, self.landingModeCallback)
        self.offboardModeSubscriber             = rospy.Subscriber("/offboard", Bool, self.offboardModeCallback)
        self.velocityModeSubscriber             = rospy.Subscriber("/velocity_mode", Bool, self.velocityModeCallback)
        # ------------------------------------- #
        # ************SERVICES***************** #
        # ------------------------------------- #
                    # ARMING
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
                    # SET MODE
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        self.pose = PoseStamped()

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.localPositionPublisher.publish(self.pose)
            self.rate.sleep()

        self.takeOffModeSet = SetModeRequest()
        self.takeOffModeSet.custom_mode = 'AUTO.TAKEOFF'
        
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.offboardModeSet = SetModeRequest()
        self.offboardModeSet.custom_mode = "OFFBOARD"

        self.landInOffboard = SetModeRequest()
        self.landInOffboard.custom_mode = "AUTO.LAND"

        self.disarm_cmd = CommandBoolRequest()
        self.disarm_cmd.value = True

        print("UAV{0} Offboard Class Initialized".format(self.uav_ID))

    def stateCallback(self, data):
        self.current_state = data

    def targetVelocityCallback(self, data):
        self.targetVelocity = data

    def takeOffModeCallback(self, data):
        self.takeOffMode = data.data

    def landingModeCallback(self, data):
        self.landingMode = data.data

    def offboardModeCallback(self, data):
        self.offboardMode = data.data
    
    def velocityModeCallback(self, data):
        self.velocityMode = data.data

    def heightCallback(self, data):
        self.height = data

    def offboard(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if (self.takeOffMode and (not self.landingMode) and (not self.offboardMode) and self.current_state.connected and (self.current_state.system_status == 3)):
                if ((self.current_state.mode != "AUTO.TAKEOFF") and (rospy.Time.now() - self.lastTime) > rospy.Duration(5.0)):
                    if (self.set_mode_client.call(self.takeOffModeSet).mode_sent == True):
                        rospy.loginfo("Takeoff Mode Set")
                    self.lastTime = rospy.Time.now()
                else:
                    if ((not self.current_state.armed) and (rospy.Time.now() - self.lastTime) > rospy.Duration(5.0)):
                        if (self.arming_client.call(self.arm_cmd).success == True):
                            rospy.loginfo("Vehicle armed")
                        self.lastTime = rospy.Time.now()

            if (self.takeOffMode and (not self.landingMode) and (self.offboardMode) and self.current_state.connected and (self.current_state.system_status == 4)):
                if ((self.current_state.mode != "OFFBOARD") and (rospy.Time.now() - self.lastTime) > rospy.Duration(5.0)):
                    # Send a few setpoints before starting
                    for i in range(100):   
                        if(rospy.is_shutdown()):
                            break
                        self.localPositionPublisher.publish(self.pose)
                        self.rate.sleep()
                    if (self.set_mode_client.call(self.offboardModeSet).mode_sent == True):
                        rospy.loginfo("Offboard enabled")
                        
                    self.lastTime = rospy.Time.now()
                if not self.velocityMode:
                    self.pose.pose.position.x = 0
                    self.pose.pose.position.y = 0
                    self.pose.pose.position.z = self.height.z
                    self.localPositionPublisher.publish(self.pose)
                if self.velocityMode:
                    self.localVelocityPublisher.publish(self.targetVelocity)
                    

            if ((not self.takeOffMode) and self.landingMode and self.current_state.connected and (self.current_state.system_status == 4)):
                if ((self.current_state.mode != "AUTO.LAND") and (rospy.Time.now() - self.lastTime) > rospy.Duration(5.0)):
                    if (self.set_mode_client.call(self.landInOffboard).mode_sent == True):
                        rospy.loginfo("Landing Enabled")
                    self.lastTime = rospy.Time.now()
                if ((not self.current_state.armed) and rospy.Time.now() - self.lastTime > rospy.Duration(5.0)):
                    if (self.arming_client.call(self.disarm_cmd).success == True):
                        rospy.loginfo("Vehicle Dis-armed")
                        return 
                    self.lastTime = rospy.Time.now()
        

if __name__ == '__main__':
    myobject = OFFBOARD(0)
    try:
        myobject.offboard()
    except rospy.ROSInterruptException:
        pass