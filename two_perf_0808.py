# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import path

from numpy import sqrt
import time
import math

import csv

###############################################
## input variables
###############################################
def uav_init(curr_speed):
    # initial uav position
    xa0 = (2*np.random.rand(1)-1)*100 #[m] 
    ya0 = (2*np.random.rand(1)-1)*100 #[m]
    xa0 = xa0[0]
    ya0 = ya0[0]
    
    # initial uav velocity
    tha0 = np.random.rand(1)*2*np.pi #[radian]
    tha0 = tha0[0]
    current_speed = curr_speed # 25 #[m/s]
    vxa0 = abs(current_speed*np.cos(tha0))  # to make circles in the samw direction
    vya0 = abs(current_speed*np.sin(tha0))
    
    return xa0,ya0,vxa0,vya0



def distance(x0,x1):
    result = math.sqrt( math.pow(x0[0]-x1[0], 2) + math.pow(x0[1]-x1[1], 2))
    return result

def  uav_optimal_tracking_control(state_aircraft,state_tracking):

    xa0 = state_aircraft[0] 
    ya0 = state_aircraft[1]
    vxa0 = state_aircraft[2]
    vya0 = state_aircraft[3]
    ux_min = state_aircraft[4]
    ux_max = state_aircraft[5]
    uy_min = state_aircraft[6]
    uy_max = state_aircraft[7]
    v_min = state_aircraft[8] 
    v_max = state_aircraft[9]
    
    current_speed = np.sqrt(vxa0**2+vya0**2)
    
    xt0 = state_tracking[0]
    yt0 = state_tracking[1]
    w_max = state_tracking[2]
    r_min = state_tracking[3] 
    n_sample = state_tracking[4]
    Dt = state_tracking[5]

    J_cost_uxuy0_function = lambda Dt,ux0,uy0,vxa0,vya0,w_max,xa0,xt0,ya0,yt0: 1.0*Dt**4*ux0**2 + 1.0*Dt**4*uy0**2 + 4.0*Dt**3*ux0*vxa0 + 4.0*Dt**3*uy0*vya0 + 2.0*Dt**2*ux0*xa0 - 2.0*Dt**2*ux0*xt0 - 0.333333333333333*Dt**2*ux0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*Dt**2*uy0*ya0 - 2.0*Dt**2*uy0*yt0 - 0.333333333333333*Dt**2*uy0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 5.0*Dt**2*vxa0**2 + 5.0*Dt**2*vya0**2 + 6.0*Dt*vxa0*xa0 - 6.0*Dt*vxa0*xt0 - 1.0*Dt*vxa0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 6.0*Dt*vya0*ya0 - 6.0*Dt*vya0*yt0 - 1.0*Dt*vya0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*xa0**2 - 4.0*xa0*xt0 - 0.666666666666667*xa0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2*xt0**2 + 0.666666666666667*xt0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*ya0**2 - 4.0*ya0*yt0 - 0.666666666666667*ya0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2*yt0**2 + 0.666666666666667*yt0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*(0.333333333333333*Dt**3*ux0*w_max + Dt**2*vxa0*w_max + 0.666666666666667*Dt*w_max*xa0 - 0.666666666666667*Dt*w_max*xt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*(0.333333333333333*Dt**3*uy0*w_max + Dt**2*vya0*w_max + 0.666666666666667*Dt*w_max*ya0 - 0.666666666666667*Dt*w_max*yt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)
        
    # body and global coordinates transfrom dcm
    th_flight = np.arctan2(vya0,vxa0);
    dcm_from_body_to_global = np.array([[np.cos(th_flight), -np.sin(th_flight)], 
                                        [np.sin(th_flight), np.cos(th_flight)]])

    # check the curvature constraint in the body frame
    u_curvature = current_speed**2/r_min
    if u_curvature < uy_max:
        # active constraint & replace the uy bound
        uy_max = u_curvature
        uy_min = -u_curvature
    
    # active the maximum velocity constraint
    vmax_active = False
    if current_speed/Dt+ux_max > v_max/Dt:
        ux_max = -current_speed/Dt+np.sqrt((v_max/Dt)**2-uy_max**2)
        vmax_active = True
    
    # active the minimum velocity constraint
    vmin_active = False
    if current_speed/Dt+ux_min < v_min/Dt:
        ux_min = -current_speed/Dt+np.sqrt((v_min/Dt)**2-uy_max**2)
        vmin_active = True

    # find the optimal solution along the boundary
    n_sample = 50
    ux_sample = np.linspace(ux_min, ux_max, n_sample)
    upper_line = np.vstack((ux_sample,np.ones(n_sample)*uy_max))
    lower_line = np.vstack((ux_sample,np.ones(n_sample)*uy_min))
    
    if vmax_active:
        ux_sample = np.linspace(ux_max,(v_max-current_speed)/Dt,n_sample)
        uy_sample = np.sqrt((v_max/Dt)**2-(ux_sample+current_speed/Dt)**2)
        right_line = np.vstack((np.hstack((ux_sample,np.flip(ux_sample))), np.hstack((uy_sample,-np.flip(uy_sample)))))
    else:
        uy_sample = np.linspace(uy_min,uy_max,n_sample)
        right_line = np.vstack((np.ones(n_sample)*ux_max, uy_sample))
        
    if vmin_active:
        ux_sample = np.linspace(ux_min,(v_min-current_speed)/Dt,n_sample)
        uy_sample = np.sqrt((v_min/Dt)**2-(ux_sample+current_speed/Dt)**2)
        left_line = np.vstack((np.hstack((ux_sample,np.flip(ux_sample))), np.hstack((uy_sample,-1*np.flip(uy_sample)))))
    else:
        uy_sample = np.linspace(uy_min,uy_max,n_sample)
        left_line = np.vstack((np.ones(n_sample)*ux_min, uy_sample))
    
    all_samples_in_body_frame = np.hstack((upper_line,lower_line,right_line,left_line))
    all_samples_in_global_frame = dcm_from_body_to_global@all_samples_in_body_frame
    ux0_sample = all_samples_in_global_frame[0,:]
    uy0_sample = all_samples_in_global_frame[1,:]
    J_val = J_cost_uxuy0_function(Dt,ux0_sample,uy0_sample,vxa0,vya0,w_max,xa0,xt0,ya0,yt0)

    
    J_val_opt = J_val.min()
    opt_idx = J_val.argmin()
    uxy_opt_body = all_samples_in_body_frame[:,opt_idx]
    uxy_opt_global = all_samples_in_global_frame[:,opt_idx]
    
    # check the cost function inside the constraint
    polygon_points = np.vstack((ux0_sample,uy0_sample))
    polygon_center = polygon_points.mean(axis=1)
    pc_vector = polygon_points-polygon_center[:,np.newaxis]
    th_pc = np.arctan2(pc_vector[1,:],pc_vector[0,:])
    idx_pc = th_pc.argsort()
    polygon_points = polygon_points[:,idx_pc]
    polygon = path.Path(polygon_points.transpose())
    
    n_inside_sample = 1000;
    x_sample = polygon_points[0,:].min() \
        + (polygon_points[0,:].max()-polygon_points[0,:].min())*np.random.rand(n_inside_sample)
    y_sample = polygon_points[1,:].min() \
        + (polygon_points[1,:].max()-polygon_points[1,:].min())*np.random.rand(n_inside_sample)
    xy_sample=np.vstack((x_sample,y_sample)).transpose()
    
    in_out = polygon.contains_points(xy_sample)
    x_sample = x_sample[in_out]
    y_sample = y_sample[in_out]
    J_val_inside = J_cost_uxuy0_function(Dt,x_sample,y_sample,vxa0,vya0,w_max,xa0,xt0,ya0,yt0)
    J_val_inside = J_val_inside[J_val_inside<J_val_opt]

    if J_val_inside.shape[0]!=0:
        J_val_opt = J_val_inside.min()
        min_idx = J_val_inside.argmin()
          
        dJdux0 = lambda Dt,ux0,uy0,vxa0,vya0,w_max,xa0,xt0,ya0,yt0: -0.666666666666667*Dt**5*ux0*w_max/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*Dt**4*ux0 + 0.111111111111111*Dt**4*ux0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 0.111111111111111*Dt**4*uy0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 2.0*Dt**4*vxa0*w_max/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 4.0*Dt**3*vxa0 + 0.333333333333333*Dt**3*vxa0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 0.333333333333333*Dt**3*vya0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 1.33333333333333*Dt**3*w_max*xa0/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 1.33333333333333*Dt**3*w_max*xt0/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 1.33333333333333*Dt**3*w_max*(0.333333333333333*Dt**3*ux0*w_max + Dt**2*vxa0*w_max + 0.666666666666667*Dt*w_max*xa0 - 0.666666666666667*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*Dt**2*xa0 + 0.222222222222222*Dt**2*xa0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 2.0*Dt**2*xt0 - 0.222222222222222*Dt**2*xt0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 0.222222222222222*Dt**2*ya0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 0.222222222222222*Dt**2*yt0*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 1.33333333333333*Dt**2*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(0.333333333333333*Dt**3*ux0*w_max + Dt**2*vxa0*w_max + 0.666666666666667*Dt*w_max*xa0 - 0.666666666666667*Dt*w_max*xt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**2 - 1.33333333333333*Dt**2*(0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)*(0.333333333333333*Dt**3*uy0*w_max + Dt**2*vya0*w_max + 0.666666666666667*Dt*w_max*ya0 - 0.666666666666667*Dt*w_max*yt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**2 - 0.333333333333333*Dt**2*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)
        dJduy0 = lambda Dt,ux0,uy0,vxa0,vya0,w_max,xa0,xt0,ya0,yt0: -0.666666666666667*Dt**5*uy0*w_max/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 0.111111111111111*Dt**4*ux0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 2.0*Dt**4*uy0 + 0.111111111111111*Dt**4*uy0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 2.0*Dt**4*vya0*w_max/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 0.333333333333333*Dt**3*vxa0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 4.0*Dt**3*vya0 + 0.333333333333333*Dt**3*vya0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 1.33333333333333*Dt**3*w_max*ya0/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 1.33333333333333*Dt**3*w_max*yt0/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 1.33333333333333*Dt**3*w_max*(0.333333333333333*Dt**3*uy0*w_max + Dt**2*vya0*w_max + 0.666666666666667*Dt*w_max*ya0 - 0.666666666666667*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 0.222222222222222*Dt**2*xa0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 0.222222222222222*Dt**2*xt0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) + 2.0*Dt**2*ya0 + 0.222222222222222*Dt**2*ya0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 2.0*Dt**2*yt0 - 0.222222222222222*Dt**2*yt0*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**(3/2) - 1.33333333333333*Dt**2*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(0.333333333333333*Dt**3*ux0*w_max + Dt**2*vxa0*w_max + 0.666666666666667*Dt*w_max*xa0 - 0.666666666666667*Dt*w_max*xt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**2 - 1.33333333333333*Dt**2*(0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)*(0.333333333333333*Dt**3*uy0*w_max + Dt**2*vya0*w_max + 0.666666666666667*Dt*w_max*ya0 - 0.666666666666667*Dt*w_max*yt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)**2 - 0.333333333333333*Dt**2*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)
            
        s_amj = 0.01
        alpha_amj = s_amj; beta_amj = 0.5; sigma_amj = 1e-5
        u_xy_current = np.array([ux0_sample[min_idx], uy0_sample[min_idx]])
        J_current = J_cost_uxuy0_function(Dt,u_xy_current[0],u_xy_current[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)
        dJdu = np.array([dJdux0(Dt,u_xy_current[0],u_xy_current[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0),
                         dJduy0(Dt,u_xy_current[0],u_xy_current[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)])
        while True:
            u_xy_update = u_xy_current - alpha_amj*dJdu
            J_update = J_cost_uxuy0_function(Dt,u_xy_update[0],u_xy_update[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)
            if J_update < (J_current + sigma_amj*alpha_amj*np.sum(dJdu**2)):
                if np.linalg.norm(u_xy_current-u_xy_update)<1e-6:
                    break
                alpha_amj = s_amj
                J_current = J_cost_uxuy0_function(Dt,u_xy_update[0],u_xy_update[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)
                dJdu = np.array([dJdux0(Dt,u_xy_update[0],u_xy_update[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0),
                                 dJduy0(Dt,u_xy_update[0],u_xy_update[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)])
                u_xy_current = u_xy_update
            else:
                alpha_amj = beta_amj*alpha_amj
      
        uxy_opt_global = u_xy_current
        uxy_opt_body = dcm_from_body_to_global.T@uxy_opt_global
        
    return uxy_opt_global, uxy_opt_body, J_val_opt, opt_idx, all_samples_in_body_frame, dcm_from_body_to_global

################################################################# from here ##############################################################

def  collision_uav_optimal_tracking_control(alpha,state_ac0,state_ac1,state_tracking,J_val_opt,uxy_opt_body,all_samples_in_body_frame,dcm_from_body_to_global):

    if alpha == 0:
        return dcm_from_body_to_global@uxy_opt_body
    
    xa0 = state_ac0[0] 
    ya0 = state_ac0[1]
    
    xa1 = state_ac1[0] 
    ya1 = state_ac1[1]
    
    ac0_pos = np.array([xa0,ya0])
    ac1_pos = np.array([xa1,ya1])
    
    # additional error is allowed to avoid collision
    J_allow = J_val_opt*(1+alpha)
    
    # find a input vector on the constraint boundary(Collision Avoidance point) on the opposite direction of ac1
    vector_ac0_1 = np.array([ac0_pos[0]-ac1_pos[0], ac0_pos[1]-ac1_pos[1]]) #global
    
    cos_max = -1 # cosine similarity value should be 1 if the vectors makes 180 degree
    
    # search all sample points on the constraint box to find C.A. coordinate
    for i in range(all_samples_in_body_frame.shape[1]):
        bd_pt_body = all_samples_in_body_frame[:,i] 
        bd_pt_global = dcm_from_body_to_global@bd_pt_body 
    
        if np.linalg.norm(bd_pt_global)==0:
            continue;
        cos = np.dot(vector_ac0_1,bd_pt_global)/(np.linalg.norm(vector_ac0_1)*np.linalg.norm(bd_pt_global))
        if cos > cos_max :
            cos_max = cos
            uxy_ca_body = bd_pt_body
 
    #sample the line from C.A. point to J min point
    N_line_sample = 50
    
    ux_line_sample = np.linspace(uxy_ca_body[0],uxy_opt_body[0],N_line_sample) # body
    uy_line_sample = np.linspace(uxy_ca_body[1],uxy_opt_body[1],N_line_sample) # body            
    uxy_line_sample_body = np.vstack((np.hstack(ux_line_sample),np.hstack(uy_line_sample))) # body
    
    uxy_line_sample_global = dcm_from_body_to_global@uxy_line_sample_body # global frame
    uxy_opt_global_new = check_line(uxy_line_sample_global,state_ac0,state_tracking,J_allow)
    
    return uxy_opt_global_new 
        
def check_line(uxy_line_sample,state_ac0,state_tracking,J_allow):
    xa0 = state_ac0[0] 
    ya0 = state_ac0[1]
    vxa0 = state_ac0[2]
    vya0 = state_ac0[3]
    xt0 = state_tracking[0]
    yt0 = state_tracking[1]
    w_max = state_tracking[2]
    Dt = state_tracking[5]

    # cost func of ac0
    J_cost_uxuy0_function = lambda Dt,ux0,uy0,vxa0,vya0,w_max,xa0,xt0,ya0,yt0: 1.0*Dt**4*ux0**2 + 1.0*Dt**4*uy0**2 + 4.0*Dt**3*ux0*vxa0 + 4.0*Dt**3*uy0*vya0 + 2.0*Dt**2*ux0*xa0 - 2.0*Dt**2*ux0*xt0 - 0.333333333333333*Dt**2*ux0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*Dt**2*uy0*ya0 - 2.0*Dt**2*uy0*yt0 - 0.333333333333333*Dt**2*uy0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 5.0*Dt**2*vxa0**2 + 5.0*Dt**2*vya0**2 + 6.0*Dt*vxa0*xa0 - 6.0*Dt*vxa0*xt0 - 1.0*Dt*vxa0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 6.0*Dt*vya0*ya0 - 6.0*Dt*vya0*yt0 - 1.0*Dt*vya0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*xa0**2 - 4.0*xa0*xt0 - 0.666666666666667*xa0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2*xt0**2 + 0.666666666666667*xt0*(2.0*Dt**3*ux0*w_max + 6.0*Dt**2*vxa0*w_max + 4.0*Dt*w_max*xa0 - 4*Dt*w_max*xt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*ya0**2 - 4.0*ya0*yt0 - 0.666666666666667*ya0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2*yt0**2 + 0.666666666666667*yt0*(2.0*Dt**3*uy0*w_max + 6.0*Dt**2*vya0*w_max + 4.0*Dt*w_max*ya0 - 4*Dt*w_max*yt0)/sqrt((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*(0.333333333333333*Dt**3*ux0*w_max + Dt**2*vxa0*w_max + 0.666666666666667*Dt*w_max*xa0 - 0.666666666666667*Dt*w_max*xt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2) + 2.0*(0.333333333333333*Dt**3*uy0*w_max + Dt**2*vya0*w_max + 0.666666666666667*Dt*w_max*ya0 - 0.666666666666667*Dt*w_max*yt0)**2/((0.333333333333333*Dt**2*ux0 + Dt*vxa0 + 0.666666666666667*xa0 - 0.666666666666667*xt0)**2 + (0.333333333333333*Dt**2*uy0 + Dt*vya0 + 0.666666666666667*ya0 - 0.666666666666667*yt0)**2)
    for i in range(uxy_line_sample.shape[1]):
        uxy = uxy_line_sample[:,i] #global
        cost =J_cost_uxuy0_function(Dt,uxy[0],uxy[1],vxa0,vya0,w_max,xa0,xt0,ya0,yt0)
        if cost <= J_allow :
            return uxy
        
def next_pos(ac0_pos,vxy_global0,uxy_global0,Dt):
    return ac0_pos + vxy_global0*Dt + 0.5*uxy_global0*Dt**2

###############################################
## main part of monte-carlo simulations
###############################################
def main_func(alpha):
    
    # control acceleration input magnitude constraints
    ux_max = 10 # [m/s^2]
    ux_min = -1 # [m/s^2]
    uy_max = 2  # [m/s^2]
    uy_min = -2 # [m/s^2]
    
    # uav minimum & maximum speed
    v_min = 20  #[m/s]
    v_max = 40  #[m/s]
    
    # initial target position
    xt0 = (2*np.random.rand(1)-1)*200  #[m] 
    yt0 = (2*np.random.rand(1)-1)*200  #[m]
    
    xt0 = xt0[0]
    yt0 = yt0[0]
    
    # target maximum speed
    w_max = 60*1e3/3600 #[m/s]
    
    # uav flying path curvature constraint
    r_min = 400 #[m]
    
    # number of samples for the control search on the boundary
    n_sample = 50
    
    # time interval for the cost approximation
    Dt = 1 # [seconds]
    N_sim = 1800
    
    # initialize aircrafts ****
    xa0, ya0, vxa0, vya0 = uav_init(25)
    xa1, ya1, vxa1, vya1 = uav_init(25)
    
    xa1=xa1+500
    ya1=ya1+500
    
    # aircraft & target dynamics
    Fa = np.eye(4)+np.vstack((np.hstack((np.zeros((2,2)),Dt*np.eye(2))),np.zeros((2,4)))) 
    Ga = np.vstack((np.zeros((2,2)),Dt*np.eye(2)))
    
    Ft = np.eye(2)
    Gt = Dt*np.eye(2)
    
    state_uav0 = np.array([xa0, ya0, vxa0, vya0]).squeeze()
    state_uav1 = np.array([xa1, ya1, vxa1, vya1]).squeeze()
    state_target = np.array([xt0, yt0]).squeeze()
    
    dist_min = 50
    
    for idx_sim in np.arange(N_sim):
        
    #    print(f'{idx_sim:3d}/{N_sim:3d}\n')
    
        # uav optimal input
        state_aircraft0 = np.array([xa0,ya0,vxa0,vya0,ux_min,ux_max,uy_min,uy_max,v_min,v_max])
        state_aircraft1 = np.array([xa1,ya1,vxa1,vya1,ux_min,ux_max,uy_min,uy_max,v_min,v_max])
        state_tracking = np.array([xt0,yt0,w_max,r_min,n_sample,Dt])
        
        uxy_opt_global0, uxy_opt_body0, J_val_opt0, opt_idx0, all_samples_in_body_frame0, dcm_from_body_to_global0 = uav_optimal_tracking_control(state_aircraft0,state_tracking)
        uxy_opt_global1, uxy_opt_body1, J_val_opt1, opt_idx1, all_samples_in_body_frame1, dcm_from_body_to_global1 = uav_optimal_tracking_control(state_aircraft1,state_tracking)
    
        # target maximum random direction input
        rand_th = 2*np.pi*np.random.rand(1)
        wx0 = w_max*np.cos(rand_th)  
        wy0 = w_max*np.sin(rand_th)
        
        # position of ac0 and ac1
        ac0_pos = np.array([xa0,ya0])
        ac1_pos = np.array([xa1,ya1])
        
        # calculate current distance and store it in array
        dist = distance(ac0_pos,ac1_pos) 
        
        # say that there is collision possibility if the current distance is closer than the threshold
        if (dist < 50):
            # apply coliision function
            uxy_opt_global0 = collision_uav_optimal_tracking_control(alpha,state_aircraft0,state_aircraft1,state_tracking,J_val_opt0,uxy_opt_body0,all_samples_in_body_frame0,dcm_from_body_to_global0)
            uxy_opt_global1 = collision_uav_optimal_tracking_control(alpha,state_aircraft1,state_aircraft0,state_tracking,J_val_opt1,uxy_opt_body1,all_samples_in_body_frame1,dcm_from_body_to_global1)
            
            # judgement of the function
            if dist < dist_min:
                dist_min = dist
        # state propagation
        state_uav0 = Fa@state_uav0 + Ga@uxy_opt_global0
        state_uav1 = Fa@state_uav1 + Ga@uxy_opt_global1
        state_target = Ft@state_target + Gt@(np.array([wx0, wy0]).squeeze())
        
        # reset state variables
        xa0 = state_uav0[0]
        ya0 = state_uav0[1]
        vxa0 = state_uav0[2]
        vya0 = state_uav0[3]
        
        xa1 = state_uav1[0]
        ya1 = state_uav1[1]
        vxa1 = state_uav1[2]
        vya1 = state_uav1[3]
        
        xt0 = state_target[0]
        yt0 = state_target[1]       
  
        time.sleep(0.01)
    print(dist_min)
    return dist_min


#######main
N_sim = 1000
with open("two_result.csv", 'w') as f:
    wr = csv.writer(f)
    for alpha in range(0,50,5):    # 0, 0.005, 0.01, 0.015,...,0.05
        p = alpha/1000
        d = np.array([])
        for i in range(N_sim):
            min_dist = main_func(p)
            while min_dist == 50:
                min_dist = main_func(p)
            d = np.append(d,min_dist)
        wr.writerow([p, np.mean(d), np.var(d)])
        print([p, np.mean(d), np.var(d)])
f.close()