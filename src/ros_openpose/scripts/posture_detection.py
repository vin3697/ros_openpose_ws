#!/usr/bin/env python


# ! coomented visualizer.py node and echo.py node in core.launch inside docker container named vibrant_buck!!!
# import modules
import rospy
from ros_openpose.msg import Frame 
import math
from ros_openpose.msg import Posture
#* declare body parts 
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

threshold_confidence = 0.7
threshold_distance = 1.2

def angle_calculation(person):
    #angles = []
    rhip_x = person.bodyParts[RHip].pixel.x
    rhip_y = person.bodyParts[RHip].pixel.y

    rx_knee = person.bodyParts[RKnee].pixel.x
    ry_knee = person.bodyParts[RKnee].pixel.y

    rx_ankle = person.bodyParts[RAnkle].pixel.x
    ry_ankle = person.bodyParts[RAnkle].pixel.y

    lhip_x = person.bodyParts[LHip].pixel.x
    lhip_y = person.bodyParts[LHip].pixel.y

    lx_knee = person.bodyParts[LKnee].pixel.x
    ly_knee = person.bodyParts[LKnee].pixel.y

    lx_ankle = person.bodyParts[LAnkle].pixel.x
    ly_ankle = person.bodyParts[LAnkle].pixel.y

    try:
        r_slope1 = (rhip_y - ry_knee) / (rhip_x - rx_knee)
        r_slope2 = (ry_ankle - ry_knee) / (rx_ankle - rx_knee)
        r_angle_radians = math.atan(abs((r_slope2 - r_slope1) / (1 + r_slope1 * r_slope2)))
        r_angle_degrees = math.degrees(r_angle_radians)

        l_slope1 = (lhip_y - ly_knee) / (lhip_x - lx_knee)
        l_slope2 = (ly_ankle - ly_knee) / (lx_ankle - lx_knee)
        l_angle_radians = math.atan(abs((l_slope2 - l_slope1) / (1 + l_slope1 * l_slope2)))
        l_angle_degrees = math.degrees(l_angle_radians)
        #angles.append(r_angle_degrees, l_angle_degrees)
        return r_angle_degrees, l_angle_degrees
    except ZeroDivisionError:
        return -1 , -1




class all_posture():

    def sitting(self, person):
        r_angle_formed, l_angle_formed = angle_calculation(person)
        rhip_y = person.bodyParts[RHip].point.y
        ry_knee = person.bodyParts[RKnee].point.y
        lhip_y = person.bodyParts[LHip].pixel.y
        ly_knee = person.bodyParts[LKnee].pixel.y
        diff_ry = (abs(rhip_y - ry_knee))*100
        diff_ly = (abs(lhip_y - ly_knee))*100
        #rospy.loginfo('%d , %d\n',r_angle_formed, l_angle_formed)
        return (((r_angle_formed >= 45) or (l_angle_formed >= 45)) 
                or ((diff_ry <= 15) and (diff_ly <= 15)))
    
    def facing_away(self, person):
        return (((person.bodyParts[REar].pixel.x == 0) and (person.bodyParts[LEar].score >= threshold_confidence))
                    or ((person.bodyParts[LEar].pixel.x == 0) and (person.bodyParts[REar].score >= threshold_confidence)))
    
    def facing_back(self, person):
        return ((person.bodyParts[Nose].pixel.x == 0) and (person.bodyParts[LEye].pixel.x == 0) and (person.bodyParts[REye].pixel.x == 0))


def frame_callback(frame):

    pub_posture = rospy.Publisher('/pose_face', Posture, queue_size=10)
    pose_face_msg = Posture()
    posture = all_posture() #object of all_posture() class
    # Extract frame details
    header = frame.header
    frame_id = header.frame_id
    persons = frame.persons    # Process each person in the frame
    person_counter = 0         #for every frame we have count to be zero!

    for person in persons:

        #TODO : define custom message to give pose of person with ID
        l_ankle_score = person.bodyParts[LAnkle].score
        r_ankle_score = person.bodyParts[RAnkle].score
        neck_score    = person.bodyParts[Neck].score
        midHip_distance = person.bodyParts[MidHip].point.z
        rospy.loginfo('Person ID: %d\n', person_counter) 
        pose_face_msg.person_id = person_counter 

        if (l_ankle_score > threshold_confidence or r_ankle_score > threshold_confidence) and neck_score > threshold_confidence and midHip_distance > threshold_distance:
            
            if (posture.facing_away(person)):
                rospy.loginfo('Person is facing away!\n')
                pose_face_msg.facing = 'FacingSideways'
            elif (posture.facing_back(person)):
                rospy.loginfo('Person is facing backward!\n')
                pose_face_msg.facing = 'FacingBackwards'
            else:
                rospy.loginfo('Person is facing towards!\n')
                pose_face_msg.facing = 'FacingTowards'

            if(posture.sitting(person)):
                rospy.loginfo('Person is sitting!\n')
                pose_face_msg.pose = 'Sitting'
            else:
                rospy.loginfo('Person is standing!\n')
                pose_face_msg.pose = 'Standing'

        else:
            rospy.loginfo('Whole person is not in the frame!\n')
            pose_face_msg.pose = 'NoPose'
            pose_face_msg.facing= 'NoFace'  
        pub_posture.publish(pose_face_msg)
        person_counter +=1
            


def frame_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber('/frame', Frame, frame_callback)  
    rospy.spin()

if __name__ == '__main__':
    frame_subscriber()
    