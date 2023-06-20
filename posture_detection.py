#!/usr/bin/env python


# ! coomented visualizer.py node and echo.py node in core.launch inside docker container named vibrant_buck!!!
# import modules
import rospy
from ros_openpose.msg import Frame 
import math
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


def facing_away(person):
    return ((((person.bodyParts[Nose].pixel.x == 0) and (person.bodyParts[LEye].pixel.x == 0) and (person.bodyParts[REye].pixel.x == 0)))
                    or ((person.bodyParts[REar].pixel.x == 0) and (person.bodyParts[LEar].score >= threshold_confidence))
                    or ((person.bodyParts[LEar].pixel.x == 0) and (person.bodyParts[REar].score >= threshold_confidence)))

class all_posture():

    def sitting(self, person):
        r_angle_formed, l_angle_formed = angle_calculation(person)
        rhip_y = person.bodyParts[RHip].point.y
        ry_knee = person.bodyParts[RKnee].point.y
        lhip_y = person.bodyParts[LHip].pixel.y
        ly_knee = person.bodyParts[LKnee].pixel.y
        diff_ry = (abs(rhip_y - ry_knee))*100
        diff_ly = (abs(lhip_y - ly_knee))*100
        #! add condition for angles + execution and return true 
        #TODO : add angle condition of 45` theta >= 45` and y axis distance between midHip and knee.
        #rospy.loginfo('%d , %d\n',r_angle_formed, l_angle_formed)
        return (((r_angle_formed >= 45) and (l_angle_formed >= 45)) 
                or ((diff_ry <= 18) and (diff_ly <= 18)))


def frame_callback(frame):

    posture = all_posture()

    # Extract frame details
    header = frame.header
    frame_id = header.frame_id
    persons = frame.persons
    # Process each person in the frame
    for person in persons:

        #TODO : define custom message to give pose of person with ID
        l_ankle_score = person.bodyParts[LAnkle].score
        r_ankle_score = person.bodyParts[RAnkle].score
        neck_score    = person.bodyParts[Neck].score

        if l_ankle_score > threshold_confidence and r_ankle_score > threshold_confidence and neck_score > threshold_confidence:
            away_facing   = facing_away(person)
            sitting       = posture.sitting(person)

            if (away_facing and sitting):
                rospy.loginfo('Person is sitting and facing away!\n')

            elif((not(away_facing)) and sitting):
                rospy.loginfo('Person is sitting and facing towards!\n')
            
            elif ((away_facing) and (not(sitting))):
                rospy.loginfo('Person is standing and facing away!\n')

            elif ((not(away_facing)) and (not(sitting))):
                rospy.loginfo('Person is standing and facing towards!\n')

        else:
            rospy.loginfo('Whole person is not in the frame!\n')  
            


def frame_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber('/frame', Frame, frame_callback)  
    rospy.spin()

if __name__ == '__main__':
    frame_subscriber()
    