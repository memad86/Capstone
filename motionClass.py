#! /usr/bin/env python3

import rospy, utm, sys
import numpy as np
from geometry_msgs.msg import Point
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import  Bool, Float64, Float64MultiArray
from sensor_msgs.msg import NavSatFix

class MOTION:
    def __init__(self, id):
        self.uav_ID = id
        rospy.init_node("uav_" + str(self.uav_ID) + "_motion_node", anonymous=False)
        self.rate                               = rospy.Rate(300)
        self.lastTime                           = rospy.Time.now()
        self.current_state                      = State() 
        self.targetPositionstart                = Point()
        self.targetPositionend                  = Point()
        self.myCurrentPosition                  = Point()
        self.targetVelocity                     = PositionTarget()
        self.height                             = Point()
        self.velocityMode                       = False
        self.listSet                            = False
        self.preferedVelocity                   = 0
        self.rowCount                           = 1
        self.diff                               = 0
        # ------------------------------------- #
        # ************PUBLISHERS*************** #
        # ------------------------------------- #
        self.targetVelocityPublisher            = rospy.Publisher("/target_velocity", PositionTarget, queue_size=2)

        # ------------------------------------- #
        # ************SUBSCRIBERS************** #
        # ------------------------------------- #
        self.heightSubsciber                    = rospy.Subscriber("/height", Point, self.heightCallback)
        self.targetPositionSubscriber           = rospy.Subscriber("/target_position", Float64MultiArray, self.targetPositionCallback)
        self.preferedVelocitySubscriber         = rospy.Subscriber("/prefered_velocity", Float64, self.preferedVelocityCallback)
        self.myCurrentPositionSubscriber        = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.currentPositionCallback)
                    # MODES
        self.velocityModeSubscriber             = rospy.Subscriber("/velocity_mode", Bool, self.velocityModeCallback)

        print("UAV{0} Motion Class Initialized".format(self.uav_ID))
    
    def velocityModeCallback(self, data):
        self.velocityMode = data.data
        if not self.velocityMode:
            self.listset = False

    def heightCallback(self, data):
        self.height = data
    
    def targetPositionCallback(self, data):
        difference = Point()
        self.UTM_start = utm.from_latlon(data.data[0],data.data[1])
        UTM_start = utm.from_latlon(data.data[0],data.data[1])
        UTM_end = utm.from_latlon(data.data[2],data.data[3]) 
        self.targetPositionstart.x  = UTM_start[0]
        self.targetPositionstart.y  = UTM_start[1]
        self.targetPositionend.x    = UTM_end[0]
        self.targetPositionend.y    = UTM_end[1]
        self.rowCount = data.data[4]
        difference.x = self.targetPositionend.x - self.targetPositionstart.x
        difference.y = self.targetPositionend.y - self.targetPositionstart.y 
        self.diff = difference.y / self.rowCount

    def currentPositionCallback(self, data):
        UTM_data = utm.from_latlon(data.latitude, data.longitude) 
        self.myCurrentPosition.x = UTM_data[0]
        self.myCurrentPosition.y = UTM_data[1]

    def preferedVelocityCallback(self, data):
        self.preferedVelocity = data.data
        
    def distance(self, target):
        distanceVector			= Point()
        distanceVector.x = target[0] - self.myCurrentPosition.x
        distanceVector.y = target[1] - self.myCurrentPosition.y
        DistanceNP = np.array([distanceVector.x, distanceVector.y])
        distanceVectorMag = np.linalg.norm(DistanceNP)
        print("Distance To Target ==> ", distanceVectorMag)
        if distanceVectorMag < 0.4:
            return True
        else:
            return False

    def motion(self, targetPosition):
        distanceVector			= Point()
        unitDistanceVector		= Point()
        targetVelocityVector    = Point()
        slowingDistance         = 50

        #Distance Vector from UAV to Translated Target Position  
        distanceVector.x = targetPosition[0] - self.myCurrentPosition.x
        distanceVector.y = targetPosition[1] - self.myCurrentPosition.y

        #Magnitude of Distance Vector
        DistanceNP = np.array([distanceVector.x, distanceVector.y])
        distanceVectorMag = np.linalg.norm(DistanceNP)
        print("Distance To Target ==> ", distanceVectorMag)
        

        #Unit Distance Vector
        unitDistanceVector.x = distanceVector.x / distanceVectorMag
        unitDistanceVector.y = distanceVector.y / distanceVectorMag

        #Calculate velocity Strength
        velocityStrength = self.preferedVelocity * (distanceVectorMag / slowingDistance)
        velocityStrength = min(velocityStrength, self.preferedVelocity)
        print ('vel: ', velocityStrength)
        # print (distanceVectorMag)

        #Velocity Vector Calculation
        targetVelocityVector.x = velocityStrength * unitDistanceVector.x
        targetVelocityVector.y = velocityStrength * unitDistanceVector.y

        ####################### Message Definition #################################
        self.targetVelocity.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.targetVelocity.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + \
        PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        ############################################################################
        self.targetVelocity.yaw = 0

        self.targetVelocity.velocity.x = targetVelocityVector.x
        self.targetVelocity.velocity.y = targetVelocityVector.y

        self.targetVelocity.position.z = self.height.z 
        ############################ Publishers #######################################
        self.targetVelocityPublisher.publish(self.targetVelocity) 

    def main(self):
        targetList = []
        a =[]
        while not rospy.is_shutdown():
            if self.velocityMode:
                if (not self.listSet):
                    for i in range(int(self.rowCount)):
                        if i % 2 == 0:
                            targetList.append([self.targetPositionstart.x, (self.targetPositionstart.y + (i * self.diff))])
                            targetList.append([self.targetPositionend.x, (self.targetPositionstart.y + (i * self.diff))])

                            # targetList.append([(self.targetPositionstart.x + (i * self.diff)), self.targetPositionstart.y])
                            # targetList.append([(self.targetPositionstart.x + (i * self.diff)), self.targetPositionend.y])

                        elif i % 2 != 0:
                            # targetList.append([(self.targetPositionstart.x + (i * self.diff)), self.targetPositionend.y])
                            # targetList.append([(self.targetPositionstart.x + (i * self.diff)), self.targetPositionstart.y])

                            targetList.append([self.targetPositionend.x, (self.targetPositionstart.y + (i * self.diff))])
                            targetList.append([self.targetPositionstart.x, (self.targetPositionstart.y + (i * self.diff))])
                    #targetList.append([]) #Home Point
                    self.listSet = True
                if self.listSet:
                    for i in range(len(targetList)):
                        print("waypoint no := ", i)
                        print(targetList)
                        # lat_lon = utm.to_latlon(targetList[i][0], targetList[i][1], self.UTM_start[2], self.UTM_start[3])
                        # a.append(lat_lon)
                        # print(a)
                        
                        while not rospy.is_shutdown():
                            if self.distance(targetList[i]):
                                print("Next WayPoint")
                                i += i
                                break
                            if not self.velocityMode:
                                break
                            self.motion(targetPosition=targetList[i])
                            # if self.distance(targetPosition):
                            #     self.motion(targetPosition)
                        self.rate.sleep()
                    #break # for bringing it back to home
if __name__ == '__main__':
    myobject = MOTION(0)
    try:
        myobject.main()
    except rospy.ROSInterruptException:
        sys.exit()
