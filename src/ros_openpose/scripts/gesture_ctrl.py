#!/usr/bin/env python


# ! coomented visualizer.py node and echo.py node in core.launch inside docker container named vibrant_buck!!!
# import modules
import rospy
from ros_openpose.msg import Frame , Gesture
from geometry_msgs.msg import Twist
import time

#* declare all body parts 
Neck        = 1 
Nose        = 0

REye        = 15 #Right Eye
REar        = 17 #Right Ear
RShoulder   = 2 # Right Shoulder
RElbow      = 3 # Right Elbow
RWrist      = 4 # Right wrist
RHip        = 9 # Right Hip
RKnee       = 10 #Right Knee
RAnkle      = 11# Right Ankle


MidHip      = 8 # Mid Hip as center of persons body


LEye        = 16# Left eye
LEar        = 18# Left Ear
LShoulder   = 5 # Left  LShoulder
LElbow      = 6 # Left Elbow
LWrist      = 7 # Left Wrist
LHip        = 12# Left Hip 
LKnee       = 13# Left Knee
LAnkle      = 14# Left Ankle

max_linear_vel  = 0.1
max_angular_vel = 0.1

obey_me     = False #Boolean value, to decide whether to follow the persons command or not

class movements():
    def __init__(self):

        #! rospy.init_node('robot_control', anonymous=True) https://answers.ros.org/question/371673/error-rospyinit_node-has-already-been-called-with-different-arguments/
        self.pub = rospy.Publisher('/tb_cmd_vel', Twist, queue_size=10) #? tb_cmd_vel rostopic for BOT
        self.rate = rospy.Rate(10)  # 10Hz publishing rate

    def forward_movement(self):
        twist = Twist() 
        twist.linear.x = max_linear_vel
        twist.linear.y = 0  #? to lateral movement but Bot doesnt support this movement
        twist.angular.z = 0 #? no angular movement
        self.pub.publish(twist)
        #self.rate.sleep() #! goes into infinte sleep

    def backward_movement(self):
        twist = Twist() 
        twist.linear.x = -max_linear_vel
        twist.linear.y = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -max_angular_vel
        twist.linear.x = 0
        twist.linear.y = 0
        self.pub.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = max_angular_vel
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def forward_right_turn(self):
        twist = Twist()
        twist.linear.x = max_linear_vel
        twist.linear.y = 0
        twist.angular.z = -max_angular_vel
        self.pub.publish(twist)

    def forward_left_turn(self):
        twist = Twist()
        twist.linear.x = max_linear_vel
        twist.linear.y = 0
        twist.angular.z = max_angular_vel
        self.pub.publish(twist)

class all_Gestures():
    #! add probability and distance conditon in each statement accordingly!! 
    def go_forward_gesture(self, person):
        return ((((person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                    or (person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0))
                    and not ((person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)          #not both hand raised!
                            and (person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0)))
                    and ((person.bodyParts[RShoulder].pixel.x > person.bodyParts[RWrist].pixel.x > 0)
                        or (person.bodyParts[LWrist].x > person.bodyParts[LShoulder].pixel.x > 0))
                    and ((abs(person.bodyParts[RWrist].pixel.x - person.bodyParts[RElbow].pixel.x) < 100)           # this is the width/distance on x axis
                        or (abs(person.bodyParts[LWrist].pixel.x - person.bodyParts[LElbow].pixel.x) < 100)))       # !elbow and wrist should be in line   


    def go_backward_gesture(self, person):

        return ((person.bodyParts[RElbow].pixel.y > person.bodyParts[RShoulder].pixel.y > 0) 
                    and (person.bodyParts[RElbow].pixel.y > person.bodyParts[RWrist].pixel.y > 0)
                    and (person.bodyParts[LWrist].pixel.x > person.bodyParts[Neck].pixel.x > person.bodyParts[RWrist].pixel.x)
                    and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LShoulder].pixel.y > 0 )
                    and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                    and (person.bodyParts[LWrist].pixel.x - person.bodyParts[RWrist].pixel.x) < (person.bodyParts[LShoulder].pixel.x - person.bodyParts[RShoulder].pixel.x))

    def disobey_gesture(self, person):

        return ((person.bodyParts[RElbow].pixel.y > person.bodyParts[RShoulder].pixel.y > 0) 
                    and (person.bodyParts[RElbow].pixel.y > person.bodyParts[RWrist].pixel.y > 0)
                    and (person.bodyParts[LWrist].pixel.x > person.bodyParts[Neck].pixel.x > person.bodyParts[RWrist].pixel.x)
                    and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LShoulder].pixel.y > 0 )
                    and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                    and (person.bodyParts[LWrist].pixel.x - person.bodyParts[RWrist].pixel.x) > (person.bodyParts[LShoulder].pixel.x - person.bodyParts[RShoulder].pixel.x))


    def go_right_gesture(self, person):
        return ((0 < person.bodyParts[Neck].pixel.x < person.bodyParts[RWrist].pixel.x)
                    and (0 < person.bodyParts[LShoulder].pixel.x < person.bodyParts[LElbow].pixel.x < person.bodyParts[LWrist].pixel.x)
                    and (0 < person.bodyParts[LShoulder].pixel.y < person.bodyParts[LWrist].pixel.y)
                    and (0 < person.bodyParts[RShoulder].pixel.y < person.bodyParts[RWrist].pixel.y))


    def go_left_gesture(self, person):
        return ((0 < person.bodyParts[LWrist].pixel.x < person.bodyParts[Neck].pixel.x)
                    and (0 < person.bodyParts[RWrist].pixel.x < person.bodyParts[RElbow].pixel.x < person.bodyParts[RShoulder].pixel.x)
                    and (0 < person.bodyParts[LShoulder].pixel.y < person.bodyParts[LWrist].pixel.y)
                    and (0 < person.bodyParts[RShoulder].pixel.y < person.bodyParts[RWrist].pixel.y))

    def go_forward_with_right_turn(self, person):
        return (0 < person.bodyParts[RShoulder].pixel.y < person.bodyParts[Neck].pixel.y < person.bodyParts[LShoulder].pixel.y
            and 0 < person.bodyParts[RHip].pixel.x < person.bodyParts[RShoulder].pixel.x
            and 0 < person.bodyParts[LHip].pixel.x < person.bodyParts[Neck].pixel.x)

    
    def go_forward_with_left_turn(self, person):
        return (0 < person.bodyParts[LShoulder].pixel.y < person.bodyParts[Neck].pixel.y < person.bodyParts[RShoulder].pixel.y
            and 0 < person.bodyParts[LShoulder].pixel.x < person.bodyParts[LHip].pixel.x
            and 0 < person.bodyParts[Neck].pixel.x < person.bodyParts[RHip].pixel.x)
    



def frame_callback(frame):


    #!create the object instance of class Gestures() and movements()
    Gestures = all_Gestures()
    robot_movement = movements()


    pub_gesture = rospy.Publisher('/gesture', Gesture, queue_size=10)
    #rate = rospy.Rate(10)  # 10Hz
    gesture_msg = Gesture() #instance for Gesture msg


    # Extract frame details
    header = frame.header
    #frame_id = header.frame_id
    persons = frame.persons
    
    # Process each person in the frame
    for person in persons:
        body_parts  = person.bodyParts

        #Mid Hip keypoint is consider as a center of person body
        MidHip_distance = body_parts[MidHip].point.z
        MidHip_score    = body_parts[MidHip].score

        threshold_confidence = 0.6
        near_distance       = 1 # one meter
        far_distance        = 4 # two meter
        
        if(((person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)          #both hand raised so the robot starts to follow commands!
                            and (person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0))):
            global obey_me
            obey_me = True
            rospy.loginfo('\nobey me')
            #rospy.loginfo(bool(obey_me))
            
        
        if obey_me:
            if ((person.bodyParts[Nose].pixel.x)*(person.bodyParts[LEye].pixel.x)*(person.bodyParts[REye].pixel.x)
                *(person.bodyParts[REar].pixel.x)*(person.bodyParts[LEar].pixel.x)) != 0: #the person has full attention
            
                if( (MidHip_score > threshold_confidence) and (near_distance < MidHip_distance < far_distance) ): #this criteria to choose a target person to whom bot will follow!
                    #rospy.loginfo('condition works!\n')

                    if Gestures.go_forward_gesture(person): #if the condition defined in raise_hand fucntion is true if condition will get executed!
                        # call fucntion to move robot
                        rospy.loginfo('GO Forward!\n')
                        gesture_msg.gesture = "Forward"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.forward_movement()

                    elif Gestures.disobey_gesture(person):

                        gesture_msg.gesture = "DontObey"
                        pub_gesture.publish(gesture_msg)
                        obey_me = False
                        rospy.loginfo('\n Dont obey')
                        #robot_movement.stop()
                    
                    elif Gestures.go_right_gesture(person):

                        rospy.loginfo('GO GO GO Right!\n')
                        gesture_msg.gesture = "GORight"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.turn_right()
                    
                    elif Gestures.go_left_gesture(person):

                        rospy.loginfo('GO GO GO Left \n')
                        gesture_msg.gesture = "GOLeft"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.turn_left()
                    
                    elif Gestures.go_backward_gesture(person):

                        rospy.loginfo('GO Backward \n')
                        gesture_msg.gesture = "GOBackward"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.backward_movement()

                    elif Gestures.go_forward_with_right_turn(person):

                        rospy.loginfo('forward with right turn \n')
                        gesture_msg.gesture = "Forward+R_Turn"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.forward_right_turn()

                    elif Gestures.go_forward_with_left_turn(person):

                        rospy.loginfo('forward with left turn \n')
                        gesture_msg.gesture = "Forward+L_Turn"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.forward_left_turn()
                    else:
                        #rospy.loginfo('STOP!!!!\n')
                        gesture_msg.gesture = "NoGesture"
                        pub_gesture.publish(gesture_msg)
                        robot_movement.stop()




def frame_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber('/frame', Frame, frame_callback)  
    rospy.spin()

if __name__ == '__main__':
    frame_subscriber()
    
