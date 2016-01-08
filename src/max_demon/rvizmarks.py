import rospy
from geometry_msgs.msg import PointStamped
import geometry_msgs.msg as GM
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM

import numpy as np
import copy

from max_demon.constants import *



def setup_marks():
    # mass marker
    mass_marker = VM.Marker()
    mass_marker.action = VM.Marker.ADD
    mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
    mass_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
    mass_marker.lifetime = rospy.Duration(4*DT)
    mass_marker.scale = GM.Vector3(*[0.2, 0.2, 0.2])
    mass_marker.type = VM.Marker.SPHERE
    mass_marker.id = 0
    # link marker
    link_marker = copy.deepcopy(mass_marker)
    link_marker.type = VM.Marker.LINE_STRIP
    link_marker.color = ColorRGBA(*[0.1, 0.1, 1.0, 1.0])
    link_marker.scale = GM.Vector3(*[0.05, 0.2, 0.2])
    link_marker.id = 1
    #cart marker
    cart_marker = copy.deepcopy(mass_marker)
    cart_marker.type = VM.Marker.CUBE
    cart_marker.color = ColorRGBA(*[0.1, 0.5, 1.0, 0.9])
    cart_marker.scale = GM.Vector3(*[0.2, 0.2, 0.2])
    cart_marker.id = 2
    #sac marker
    sac_marker = copy.deepcopy(cart_marker)
    sac_marker.type = VM.Marker.LINE_STRIP
    sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0])
    sac_marker.lifetime = rospy.Duration(DT)
    sac_marker.scale = GM.Vector3(*[0.1, 0.15, 0.1])
    p1 = np.array([0.0,0.0,0.1])
    p2 = np.array([0.0,0.3,0.65])
    p3 = np.array([0.0,-0.25,0.4])
    sac_marker.points = [GM.Point(*p3), GM.Point(*p1), GM.Point(*p2)]
    sac_marker.id = 3
    # score marker
    score_marker = copy.deepcopy(mass_marker)
    score_marker.type = VM.Marker.TEXT_VIEW_FACING
    score_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
    score_marker.scale = GM.Vector3(*[0.3, 0.3, 0.3])
    score_marker.pose.position.x = 0;
    score_marker.pose.position.y = 0;
    score_marker.pose.position.z = 1.0;
    score_marker.pose.orientation.x = 0.0;
    score_marker.pose.orientation.y = 0.0;
    score_marker.pose.orientation.z = 0.2;
    score_marker.pose.orientation.w = 1.0;
    score_marker.text = "0%"
    score_marker.id = 4
    #arrow marker
    dir_marker = copy.deepcopy(mass_marker)
    dir_marker.type = VM.Marker.ARROW
    dir_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0])
    dir_marker.lifetime = rospy.Duration(5*DT)
    dir_marker.scale = GM.Vector3(*[0.025, 0.05, 0.025])
    dir_marker.id = 5
    return [mass_marker, link_marker, cart_marker, sac_marker,score_marker, dir_marker]

def pub_point(g):
    p = PointStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = SIMFRAME
    ptrans = g[0:3, -1]
    p.point.x = ptrans[0]
    p.point.y = ptrans[1]
    p.point.z = ptrans[2]
    return [p,ptrans]
    
