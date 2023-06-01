#!/usr/bin/env python


# ! coomented visualizer.py node and echo.py node in core.launch inside docker container named vibrant_buck!!!
# import modules
import rospy
from ros_openpose.msg import Frame 
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




#! add probability and distance conditon in each statement accordingly!! 
def go_forward_gesture(person):
    return ((((person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                 or (person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0))
                and not ((person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)          #not both hand raised!
                         and (person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0)))
                and ((person.bodyParts[RShoulder].pixel.x > person.bodyParts[RWrist].pixel.x > 0)
                     or (person.bodyParts[LWrist].x > person.bodyParts[LShoulder].pixel.x > 0))
                and ((abs(person.bodyParts[RWrist].pixel.x - person.bodyParts[RElbow].pixel.x) < 100)           # this is the width/distance on x axis
                    or (abs(person.bodyParts[LWrist].pixel.x - person.bodyParts[LElbow].pixel.x) < 100)))       # !elbow and wrist should be in line   


def go_backward_gesture(person):

    return ((person.bodyParts[RElbow].pixel.y > person.bodyParts[RShoulder].pixel.y > 0) 
				and (person.bodyParts[RElbow].pixel.y > person.bodyParts[RWrist].pixel.y > 0)
                and (person.bodyParts[LWrist].pixel.x > person.bodyParts[Neck].pixel.x > person.bodyParts[RWrist].pixel.x)
                and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LShoulder].pixel.y > 0 )
                and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                and (person.bodyParts[LWrist].pixel.x - person.bodyParts[RWrist].pixel.x) < (person.bodyParts[LShoulder].pixel.x - person.bodyParts[RShoulder].pixel.x))

def stop_gesture(person):

    return ((person.bodyParts[RElbow].pixel.y > person.bodyParts[RShoulder].pixel.y > 0) 
				and (person.bodyParts[RElbow].pixel.y > person.bodyParts[RWrist].pixel.y > 0)
                and (person.bodyParts[LWrist].pixel.x > person.bodyParts[Neck].pixel.x > person.bodyParts[RWrist].pixel.x)
                and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LShoulder].pixel.y > 0 )
                and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
                and (person.bodyParts[LWrist].pixel.x - person.bodyParts[RWrist].pixel.x) > (person.bodyParts[LShoulder].pixel.x - person.bodyParts[RShoulder].pixel.x))


    # return ((person.bodyParts[RElbow].pixel.y > person.bodyParts[RShoulder].pixel.y > person.bodyParts[RWrist].pixel.y > 0)     # RElbow > RShoulder and RElbow > Rwrist 
    #             and (person.bodyParts[LWrist].pixel.x > person.bodyParts[Neck].pixel.x > person.bodyParts[RWrist].pixel.x)
    #             and (person.bodyParts[LElbow].pixel.y > person.bodyParts[LShoulder].pixel.y > person.bodyParts[LWrist].pixel.y > 0)
    #             and (person.bodyParts[LWrist].pixel.x - person.bodyParts[RWrist].pixel.x) > (person.bodyParts[LShoulder].pixel.x - person.bodyParts[RShoulder].pixel.x))


def go_right_gesture(person):
    return ((0 < person.bodyParts[Neck].pixel.x < person.bodyParts[RWrist].pixel.x)
                and (0 < person.bodyParts[LShoulder].pixel.x < person.bodyParts[LElbow].pixel.x < person.bodyParts[LWrist].pixel.x)
                and (0 < person.bodyParts[LShoulder].pixel.y < person.bodyParts[LWrist].pixel.y)
                and (0 < person.bodyParts[RShoulder].pixel.y < person.bodyParts[RWrist].pixel.y))


def go_left_gesture(person):
    return ((0 < person.bodyParts[LWrist].pixel.x < person.bodyParts[Neck].pixel.x)
                and (0 < person.bodyParts[RWrist].pixel.x < person.bodyParts[RElbow].pixel.x < person.bodyParts[RShoulder].pixel.x)
                and (0 < person.bodyParts[LShoulder].pixel.y < person.bodyParts[LWrist].pixel.y)
                and (0 < person.bodyParts[RShoulder].pixel.y < person.bodyParts[RWrist].pixel.y))


def frame_callback(frame):
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

        threshold_confidence = 0.7
        near_distance       = 1 # one meter
        far_distance        = 4 # four meter

        if( (MidHip_score > threshold_confidence) and (near_distance < MidHip_distance < far_distance) ): #this criteria to choose a target person to whom bot will follow!

            if go_forward_gesture(person): #if the condition defined in raise_hand fucntion is true if condition will get executed!
                #TODO : create a fucntion where bot moves forward!
                # call fucntion to move robot
                rospy.loginfo('GO Forward!\n')

            elif stop_gesture(person):

                rospy.loginfo('STOP!!!!\n')
            
            elif go_right_gesture(person):

                rospy.loginfo('GO GO GO Right!\n')
            
            elif go_left_gesture(person):

                rospy.loginfo('GO GO GO Left \n')
            
            elif go_backward_gesture(person):

                rospy.loginfo('GO Backward \n')


def frame_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber('/frame', Frame, frame_callback)  
    rospy.spin()


if __name__ == '__main__':
    frame_subscriber()


            #TODO -> below to define gestures and commnad to bot. 
            # if (body_parts[RShoulder].score > threshold_confidence and body_parts[RElbow].score > threshold_confidence and body_parts[RWrist].score > threshold_confidence):
                
            #     #* For Right Wrist
            #     x_pixel_Rwrist    = body_parts[RWrist].pixel.x
            #     y_pixel_Rwrist    = body_parts[RWrist].pixel.y
            #     x_pixel_RElbow    = body_parts[RElbow].pixel.x
            #     y_pixel_RElbow    = body_parts[RElbow].pixel.y
            #     x_pixel_RShoulder    = body_parts[RShoulder].pixel.x
            #     y_pixel_RShoulder    = body_parts[RShoulder].pixel.y
            #     rospy.loginfo('x,y pixel Rwrist %f ,%f\n' , x_pixel_Rwrist, y_pixel_Rwrist)
            #     rospy.loginfo('x,y pixel RElbow %f ,%f \n' , x_pixel_RElbow, y_pixel_RElbow)
            #     rospy.loginfo('x,y pixel RShoulder %f, %f\n' , x_pixel_RShoulder, y_pixel_RShoulder)

            #     rospy.loginfo('x,y pixel LWrist %f ,%f\n' , body_parts[LWrist].pixel.x, body_parts[LWrist].pixel.y)
            #     rospy.loginfo('x,y pixel LElbow %f ,%f \n' , body_parts[LElbow].pixel.x, body_parts[LElbow].pixel.y)
            #     rospy.loginfo('x,y pixel LShoulder %f, %f\n' , body_parts[LShoulder].pixel.x, body_parts[LShoulder].pixel.y)

            #     rospy.loginfo('x,y pixel Neck %f ,%f\n' , body_parts[Neck].pixel.x, body_parts[Neck].pixel.y)
            #     rospy.loginfo('x,y pixel MidHip %f ,%f \n' , body_parts[MidHip].pixel.x, body_parts[MidHip].pixel.y)