#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# EmptyRequest gives error but script still works when running, 
# googled it and its a problem with IDE, advised to just ignore it for now
from std_srvs.srv import Empty, EmptyRequest

# Ivan Milinkovic,2991905


class MoveRobot():
    
    def __init__(self):
        
        # publisher initializing
        self.robotVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()

        # subscriber initializing
        self.amclPoseSub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.getPose)
        self.poseInfo = PoseWithCovarianceStamped()

        # service client initializing
        rospy.wait_for_service('/global_localization')
        # spreading particles
        self.spreadParticlesSrv = rospy.ServiceProxy('/global_localization', Empty)
        self.srvRequest = EmptyRequest()

        # exit flag
        self.ctrl_c = False
        # shutdown
        rospy.on_shutdown(self.terminate)
        # rate
        self.rate = rospy.Rate(10)  # 10hz equals to 0.1 sec

    # function for termination    
    def terminate(self):
        # call stop robot function
        self.stoppingRobot()
        # flip flag
        self.ctrl_c = True

    # function for stoping robot
    def stoppingRobot(self): 
        # set speeds to 0   
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        # stop robot for 2 sec
        i = 0
        while i < 20:
            self.robotVelPub.publish(self.cmd)
            self.rate.sleep()
            i += 1

    # function to move robot ahead
    def robotAhead(self, linear_speed=0.1, angular_speed=0.0):  
        # set speeds     
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        
        # move robot ahead for 1.5 sec
        i = 0  
        while i < 15:
            self.robotVelPub.publish(self.cmd)
            self.rate.sleep()
            i += 1

    # function to turn robot
    def robotTurnRight(self, linear_speed=0.0, angular_speed=-1.55):
        # set speeds, -1.57 should turn robot 90 degrees right 
        # but while running simulation -1.55 is the closest ive come to 90 degrees    
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        
        # rotate robot for 1 sec
        i = 0   
        while i < 10:
            self.robotVelPub.publish(self.cmd)
            self.rate.sleep()
            i += 1
    
    # function to move robot in a square
    def doSquare(self):     
        # moving ahead and turning 4 time to make a square
        i = 0     
        while not self.ctrl_c and i < 4:
            # move ahead and stop
            self.robotAhead()
            self.stoppingRobot()
            # turn right and stop
            self.robotTurnRight()
            self.stoppingRobot()

            i += 1

        # once movement is done stop robot 
        self.stoppingRobot()

    # function to trigger particle spread  
    def triggerParticleSpread(self): 
        result = self.spreadParticlesSrv(self.srvRequest)
        
    # function that stores amcl pose into variable
    def getPose(self, info):
        self.poseInfo = info

    # function to calculate covariance
    def covCalculation(self):
        # we care only for values on positions 0,7,35 from pose
        cov_x = self.poseInfo.pose.covariance[0]
        cov_y = self.poseInfo.pose.covariance[7]
        cov_z = self.poseInfo.pose.covariance[35]
        cov = (cov_x+cov_y+cov_z)/3
        
        return cov


if __name__ == '__main__':
    # node init
    rospy.init_node('robotMoveNode', anonymous=True)
    # instantiate robot move class
    turtlebot = MoveRobot()
    
    # cov set to 1, anything above 0.65 would work 
    cov = 1
    
    # if cov is lower then 0.65, means robot isnt localized 
    rospy.loginfo(" ---DOING LOCALIZATION WORK--- ")
    while cov > 0.65:
        # spread particles
        turtlebot.triggerParticleSpread()
        # do square
        turtlebot.doSquare()
        # calculate cov
        cov = turtlebot.covCalculation()
        rospy.loginfo("Covariance: " + str(cov))
        if cov > 0.65:
            rospy.loginfo("Covariance is greater than 0.65, resuming with localization!")
        else:
            rospy.loginfo("Robot localized!")