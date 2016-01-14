#!/usr/bin/env python

"""
Kathleen Fitzsimons
**modified from trep_omni spherical pendulum simulation

This node runs 3 timers. (1) that looks up the TF from the base link of the omni to
the end of the stylus. It then uses the y-portion(right-left) of this pose to 
drive a trep simulation. (2) SAC timer computes the optimal action for the current
state and uses it to update the location and directionality of a virtual 'wall'. 
(3) updates the forces to render the virtual wall and calculate the SAC score.
The position of the cart and pendulum is also published.

SUBSCRIBERS:
    - omni1_button (phantom_omni/PhantomButtonEvent)

PUBLISHERS:
    - mass_point (PointStamped)
    - cart_point (PointStamped)
    - visualization_marker_array (MarkerArray)
    - omni1_force_feedback (phantom_omni/OmniFeedback)

SERVICES:

"""

################
# ROS IMPORTS: #
################
import rospy
import tf
from tf import transformations as TR
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg as GM
from phantom_omni.msg import PhantomButtonEvent
from phantom_omni.msg import OmniFeedback
from trep_omni_cart.msg import trepsys
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM


###################
# NON-ROS IMPORTS #
###################
import trep
#from trep import tx, ty, tz, rx, ry, rz
#import trep.discopt#sactrep
import numpy as np
from numpy import dot
import copy
import time
import max_demon as mda
from max_demon.constants import *
import max_demon.force_fb as fb
from max_demon import rvizmarks

class PendSimulator:

    def __init__(self):
        rospy.loginfo("Creating PendSimulator class")
        
        # define running flag:
        self.running_flag = False
        self.grey_flag = False
        self.usat = 0.
        self.sacpos = 0.
        self.sacvel = 0.
        self.prevq = np.zeros(5)
        self.prevdq = np.zeros(20)
        self.wall=0.
        self.i = 0.
        self.n = 0.
        
        # setup markers
        self.setup_markers()
        
        # setup publishers, subscribers, timers:
        self.button_sub = rospy.Subscriber("omni1_button", PhantomButtonEvent, self.buttoncb)
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.timercb)
        self.LQ_timer = rospy.Timer(rospy.Duration(TS), self.timerLQ)
        self.force_timer = rospy.Timer(rospy.Duration(DT2),self.render_forces)
        self.mass_pub = rospy.Publisher("mass_point", PointStamped, queue_size = 3)
        self.cart_pub = rospy.Publisher("cart_point", PointStamped, queue_size = 3)
        self.dir_pub = rospy.Publisher("sac_point", PointStamped, queue_size = 2)####arrow marker###
        self.trep_pub = rospy.Publisher("trep_sys", trepsys, queue_size = 3)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray, queue_size = 3)
        self.force_pub = rospy.Publisher("omni1_force_feedback", OmniFeedback , queue_size = 3)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        return

    def setup_markers(self):
        self.markers = VM.MarkerArray()
        [self.mass_marker, self.link_marker, self.cart_marker, self.sac_marker,self.score_marker, \
            self.dir_marker]=rvizmarks.setup_marks()
              
        self.markers.markers.append(self.mass_marker)
        self.markers.markers.append(self.link_marker)
        self.markers.markers.append(self.cart_marker)
        self.markers.markers.append(self.sac_marker)
        self.markers.markers.append(self.score_marker)
        self.markers.markers.append(self.dir_marker)###arrow###

        return
    
        
    def setup_integrator(self):
        self.system = mda.build_system()
        self.sactrep = mda.build_system()
        self.mvi = trep.MidpointVI(self.system)
        #[self.KStabil, self.dsys, self.xBar]=build_LQR(self.mvi, self.system)
                            
        # get the position of the omni in the trep frame
        if self.listener.frameExists(SIMFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(SIMFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(SIMFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
            return

        self.q0 = np.array([-0.1, SCALE*position[1]])#X=[th,yc]
        self.dq0 = np.zeros(self.system.nQd) 
        x0=np.array([self.q0[0],self.q0[1],0.,0.])
        
        [self.KStabil, self.dsys, self.xBar]=mda.build_LQR(self.mvi, self.system, x0)
        
        self.mvi.initialize_from_state(0, self.q0, self.dq0)
        self.u=self.mvi.q1[1]
        #compute LQR control
        x=np.array([self.system.q[0],self.system.q[1],self.system.dq[0],self.system.dq[1]])
        xTilde = x - self.xBar # Compare to desired state
        utemp = -dot(self.KStabil, xTilde) # Calculate input
        utemp = fb.sat_u(utemp-self.u)
        self.u=utemp+self.u
        
        #convert kinematic acceleration to new velocity&position
        self.sacvel = utemp/DT
        self.sacpos = self.u        
        self.wall = SCALE*position[1]
        #reset score values
        self.i = 0.
        self.n = 0.
        self.prevq = np.zeros(5)
        self.prevdq = np.zeros(20)
        return

    def timercb(self, data):
        if not self.running_flag:
            return
        if self.listener.frameExists(SIMFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(SIMFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(SIMFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
            return
        
        #update position and velocity arrays
        self.prevq = np.insert(self.prevq,0, SCALE*position[1])
        self.prevq = np.delete(self.prevq, -1)
        self.prevdq = np.insert(self.prevdq,0, self.system.dq[1])
        self.prevdq = np.delete(self.prevdq, -1)
        
        # now we can use this position to integrate the trep simulation:
        ucont = np.zeros(self.mvi.nk)
        ucont[self.system.kin_configs.index(self.system.get_config('yc'))] = self.prevq[0]
        
        # step integrator:
        try:
            self.mvi.step(self.mvi.t2 + DT,k2=ucont)
        except trep.ConvergenceError as e:
            rospy.loginfo("Could not take step: %s"%e.message)
            return
        temp = trepsys()
        temp.sys_time = self.system.t
        temp.theta = self.system.q[0]
        temp.y = self.system.q[1]
        temp.dtheta = self.system.dq[0]
        temp.dy = self.system.dq[1] #np.average(self.prevdq)
        temp.sac = self.u
        self.trep_pub.publish(temp)
        
        
        
        gwm = self.system.get_frame(MASSFRAME).g()
        [p,ptrans]=rvizmarks.pub_point(gwm)
        self.mass_pub.publish(p)
        # now we can send the transform:
        qtrans = TR.quaternion_from_matrix(gwm)
        self.br.sendTransform(ptrans, qtrans, p.header.stamp, MASSFRAME, SIMFRAME)
        ##cart sim   
        gwc = self.system.get_frame(CARTFRAME).g()
        [pc,ptransc]=rvizmarks.pub_point(gwc)
        self.cart_pub.publish(pc)
        qtransc = TR.quaternion_from_matrix(gwc)
        self.br.sendTransform(ptransc, qtransc, pc.header.stamp, CARTFRAME, SIMFRAME)
        
        ####for the arrow marker
        gwu = copy.copy(gwc)###keep
        gwu.itemset((1,3), self.sacpos)###keep
        [pu,ptransu]=rvizmarks.pub_point(gwu)
        self.dir_pub.publish(pu)###keep
        # now we can send the transform:
        qtransu = TR.quaternion_from_matrix(gwu)
        self.br.sendTransform(ptransu, qtransu, pu.header.stamp, SACFRAME, SIMFRAME)
        ####end arrow marker update
        
        # now we can publish the markers:
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptrans))
        p1 = GM.Point(*ptrans)
        p2 = GM.Point(*ptransc)
        self.link_marker.points = [p1, p2]
        self.dir_marker.points = [GM.Point(*ptransc), GM.Point(*ptransu)] ######arrow marker######
        self.cart_marker.pose = GM.Pose(position=GM.Point(*ptransc))
        self.marker_pub.publish(self.markers)
        qtemp = self.system.q
        mda.proj_func(qtemp)
        if self.system.t >= 50.0: #or abs(qtemp[0]) < 0.15 and abs(self.system.dq[0]) < 0.5: 
            rospy.loginfo("Success Time: %s"%round(self.system.t,2))
            rospy.loginfo("Final Score: %s"%round((self.i/self.n*100),2))
            self.force_pub.publish(OmniFeedback(force=GM.Vector3(), position=GM.Vector3()))
            self.running_flag = False
        return
        

    def timerLQ(self,data):
        if not self.running_flag:
            return
        #compute LQR control
        x=np.array([self.system.q[0],self.system.q[1],self.system.dq[0],self.system.dq[1]])
        xTilde = x - self.xBar # Compare to desired state
        utemp = -dot(self.KStabil, xTilde) # Calculate input
        utemp = fb.sat_u(utemp-self.u)
        self.u=utemp+self.u
        
        #convert kinematic acceleration to new velocity&position
        veltemp = utemp/DT
        self.sacpos = self.u
        #if np.sign(self.sacvel) != np.sign(veltemp):#update wall if sac changes direction
        self.wall = self.prevq[0]
        self.sacvel = veltemp
        print self.sacvel
        return
    
    def render_forces(self,data):
        if not self.running_flag:
            return
        # get the position of the stylus in the omni's base frame
        if self.listener.frameExists(BASEFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(BASEFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(BASEFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(BASEFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(BASEFRAME,CONTFRAME))
            return
        #get force magnitude
        fsac = np.array([0.,0.,0.])#np.array([0.,fb.sat_func(np.average(self.prevdq)),0.])
        if (self.sacvel > 0 and SCALE*position[1] < self.wall) or \
           (self.sacvel < 0 and SCALE*position[1] > self.wall):
            fsac = fsac+fb.wall_func(self.wall,self.prevq)
            self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 0.0])
        elif abs(SCALE*position[1] - self.prevq[1]) < SCALE*10**(-4) and self.sacvel == 0.0:
            self.sac_marker.color = ColorRGBA(*[0.05, 0.05, 1.0, 1.0])
            #self.i += 1
        #elif abs(SCALE*position[1] - self.prevq[1]) < SCALE*10**(-4):
        #   self.sac_marker.color = ColorRGBA(*[0.05, 0.05, 1.0, 0.0])
        else:
            self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0]) 
            self.i += 1 
        self.n += 1
        self.score_marker.text = "Score = "+ str(round((self.i/self.n)*100,2))+"%"
        # the following transform was figured out only through
        # experimentation. The frame that forces are rendered in is not aligned
        # with /trep_world or /base:
        fvec = np.array([fsac[1], fsac[2], fsac[0]])
        f = GM.Vector3(*fvec)
        p = GM.Vector3(*position)
        self.force_pub.publish(OmniFeedback(force=f, position=p))
        return
           
    def buttoncb(self, data):
        if data.grey_button == 1 and data.white_button == 0 and \
        self.running_flag == False:
            rospy.loginfo("Integration primed")
            self.grey_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and \
        self.grey_flag == True and self.running_flag == False:
            # then we previously pushed only the grey button, and we just released it
            rospy.loginfo("Starting integration")
            self.setup_integrator()
            self.running_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and \
        self.grey_flag == True and self.running_flag == True:
            # then the sim is already running and nothing should happend
            rospy.loginfo("Integration already running")
        elif data.white_button == 1:
            rospy.loginfo("Integration stopped")
            self.grey_flag = False
            self.running_flag = False
            self.force_pub.publish(OmniFeedback(force=GM.Vector3(), position=GM.Vector3()))
        return



def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cart_pend_sim', log_level=rospy.INFO)

    try:
        sim = PendSimulator()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
