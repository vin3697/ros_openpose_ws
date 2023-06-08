#!/usr/bin/env python


# ! coomented visualizer.py node and echo.py node in core.launch inside docker container named vibrant_buck!!!
# import modules
import rospy
from ros_openpose.msg import Frame , Gesture
from geometry_msgs.msg import Twist


#* declare all body parts 
Neck        = 1 

RShoulder   = 2 # Right Shoulder
RElbow      = 3 # Right Elbow
RWrist      = 4 # Right wrist

LShoulder   = 5 # Left  LShoulder
LElbow      = 6 # Left Elbow
LWrist      = 7 # Left Wrist

MidHip      = 8 # Mid Hip as center of persons body
RHip        = 9 # Right Hip
LHip        = 12# Left Hip 

class movements():
    def __init__(self):

        #! rospy.init_node('robot_control', anonymous=True) https://answers.ros.org/question/371673/error-rospyinit_node-has-already-been-called-with-different-arguments/
        self.pub = rospy.Publisher('/tb_cmd_vel', Twist, queue_size=10) #? tb_cmd_vel rostopic for BOT
        self.rate = rospy.Rate(10)  # 10Hz publishing rate

    def forward_movement(self):
        twist = Twist() 
        twist.linear.x = 0.1
        twist.linear.y = 0  #? to lateral movement but Bot doesnt support this movement
        twist.angular.z = 0 #? no angular movement
        self.pub.publish(twist)
        #self.rate.sleep() #! goes into infinte sleep

    def backward_movement(self):
        twist = Twist() 
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.1
        twist.linear.x = 0
        twist.linear.y = 0
        self.pub.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.1
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
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.angular.z = -0.1
        self.pub.publish(twist)

    def forward_left_turn(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.angular.z = 0.1
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

    def stop_gesture(self, person):

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


    pub = rospy.Publisher('/gesture', Gesture, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    gesture_msg = Gesture() #instance for Gesture msg


    # Extract frame details
    header = frame.header
    frame_id = header.frame_id
    persons = frame.persons
    # Process each person in the frame
    for person in persons:
        body_parts  = person.bodyParts

        #Mid Hip keypoint is consider as a center of person body
        MidHip_distance = body_parts[MidHip].point.z
        MidHip_score    = body_parts[MidHip].score

        threshold_confidence = 0.8
        near_distance       = 1 # one meter
        far_distance        = 2 # two meter

        if( (MidHip_score > threshold_confidence) and (near_distance < MidHip_distance < far_distance) ): #this criteria to choose a target person to whom bot will follow!
            #rospy.loginfo('condition works!\n')

            if Gestures.go_forward_gesture(person): #if the condition defined in raise_hand fucntion is true if condition will get executed!
                # call fucntion to move robot
                #rospy.loginfo('GO Forward!\n')
                gesture_msg.gesture = "Forward"
                pub.publish(gesture_msg)
                robot_movement.forward_movement()

            elif Gestures.stop_gesture(person):

                #rospy.loginfo('STOP!!!!\n')
                gesture_msg.gesture = "STOP"
                pub.publish(gesture_msg)
                robot_movement.stop()
            
            elif Gestures.go_right_gesture(person):

                #rospy.loginfo('GO GO GO Right!\n')
                gesture_msg.gesture = "GO Right"
                pub.publish(gesture_msg)
                robot_movement.turn_right()
            
            elif Gestures.go_left_gesture(person):

                #rospy.loginfo('GO GO GO Left \n')
                gesture_msg.gesture = "GO Left"
                pub.publish(gesture_msg)
                robot_movement.turn_left()
            
            elif Gestures.go_backward_gesture(person):

                #rospy.loginfo('GO Backward \n')
                gesture_msg.gesture = "GO Backward"
                pub.publish(gesture_msg)
                robot_movement.backward_movement()

            elif Gestures.go_forward_with_right_turn(person):

                #rospy.loginfo('forward with right turn \n')
                gesture_msg.gesture = "Forward+R_Turn"
                pub.publish(gesture_msg)
                robot_movement.forward_right_turn()

            elif Gestures.go_forward_with_left_turn(person):

                #rospy.loginfo('forward with left turn \n')
                gesture_msg.gesture = "Forward+L_Turn"
                pub.publish(gesture_msg)
                robot_movement.forward_left_turn()



def frame_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber('/frame', Frame, frame_callback)  
    rospy.spin()

if __name__ == '__main__':
    frame_subscriber()
    
