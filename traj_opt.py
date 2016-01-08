
import numpy as np
import sys
import trep
from trep import tx, ty, tz, rx, ry, rz
#from trep.discopt import
import math
from math import sin, cos
from math import pi #as mpi
from trep import visual
#import trep.visual as visual
from PyQt4.QtCore import Qt, QRectF, QPointF
from PyQt4.QtGui import QColor
#import matplotlib.pyplot as plt


class PendCartVisual(visual.VisualItem2D):
    def __init__(self, *args, **kwds):
        
        draw_track = kwds.setdefault('draw_track', True)
        del(kwds['draw_track'])

        super(PendCartVisual, self).__init__(*args, **kwds)

        if draw_track:
            self.attachDrawing(None, self.paint_track)
        self.attachDrawing('Cart', self.paint_cart)
        self.attachDrawing('PendulumBase', self.paint_pend)
        self.attachDrawing('Pendulum', self.paint_mass)

    def paint_track(self, painter):
        rect = QRectF(0, 0, 4.0, 0.05)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(100, 100, 100))

    def paint_cart(self, painter):
        rect = QRectF(0, 0, 0.2, 0.07)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(200, 200, 200))

    def paint_pend(self, painter):
        rect = QRectF(-0.01, 0, 0.02, -1.0)
        painter.fillRect(rect, QColor(0, 0, 0))

    def paint_mass(self, painter):
        rect = QRectF(0, 0, 0.07, 0.07)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(200, 200, 0))
        
      
def build_system(torque_force=False):
    cart_mass = 10.0
    pendulum_length = 2.0
    pendulum_mass = 0.1

    system = trep.System()
    frames = [
        tx('x', name='Cart', kinematic = True), [
            rz('theta', name="PendulumBase"), [
                ty(-pendulum_length, name="Pendulum", mass=pendulum_mass)]]]
    system.import_frames(frames)
    trep.potentials.Gravity(system, (0, -9.8, 0))
    trep.forces.Damping(system, 0.01)
    #trep.forces.ConfigForce(system, 'x', 'x-force')
    if torque_force:
        trep.forces.ConfigForce(system, 'theta', 'theta-force')
    return system

def generate_desired_trajectory(system, t, amp=130*pi/180):
    qd = np.zeros((len(t), system.nQ))
    theta_index = system.get_config('theta').index
    for i,t in enumerate(t):
        if t >= 0.0 and t <= 15.0:
            qd[i, theta_index] = pi#(1 - cos(2*mpi/4*(t-3.0)))*amp/2
    return qd

def generate_initial_trajectory(system, t, theta=0.):
    qd = np.zeros((len(t), system.nQ))
    theta_index = system.get_config('theta').index
    for i,t in enumerate(t):
        if t >= 0.00 and t <= 15.0:
            qd[i, theta_index] = theta
    return qd

def make_state_cost(system, dsys, base, theta,x,dtheta,dx):
    weight = base*np.ones((dsys.nX,))
    weight[system.get_config('x').index] = x
    weight[system.get_config('theta').index] = theta
    weight[system.get_config('x').index+2] = dx
    weight[system.get_config('theta').index+2] = dtheta
    return np.diag(weight)

def make_input_cost(system, dsys, base, x, theta=None):
    weight = base*np.ones((dsys.nU,))
    if theta is not None:
        weight[system.get_input('theta-force').index] = theta
    #weight[system.get_config('x').index] = x
    return np.diag(weight)     