# This physics animation was created using the Mathematical Modeling and Finite Element Analysis of the Puncture Process in Sewing Fabrics, by Mei et al., 2026)
import numpy as np
from arm import ROOT, NPZ_PATH, XML_PATH, Calibration, HandPose, MjcArm, DepthScale, ArmIK, STServo, FingerBoard

# Y - deformation of the fabric along the needle axis (force, axis)
# p - 2D planar unit pre-tension, N/m (force/area)
# E - 2D planar stiffness (resistance to deformation under tension) (Young's modulus in relation to thickness)
# r - radius of the needle plate hole in m - 1.5*needle_d
# v - a dimensionless coefficient related to Poisson’s ratio
# n - vector perpedincular to the fabric


# P = np.pi*p*Y + Y^3*(E*v^3)/r^2

  
#stage 2 sim - normal pressure before full penetration
# a - angle of the sharp end of the needle
# h - fabric thickness
# l - how far the needle has travelled
# s - cone radius

def friction_vals(endcone, startcone, cone_status):
    F = 0.47
    k = 6.6e7
    D = 0.00203
    a = np.radians(10) 

    fn = 2*np.pi*k*np.tan(a)**2 / np.cos(a)**2 * (endcone**3 - startcone**3) / 3 #fabric squeexing around the piercing needle tip
    fr = F * fn #friction from fr
    fc = k * np.pi * D**2 * cone_status/2 #shank diameter * #fabric squeezing around the needle body when it's fully in
    frc = F * fc # friction from fc
    return fn, fr, fc, frc

def fabric_force_model (sim, state, prev_tip):
    model = sim.model
    fid = model.geom("fabric_geom").id
    p0 = sim.data.geom_xpos[fid]
    Fm = sim.data.geom_xmat[fid].reshape(3, 3)

    V = 1.0 #initial value 
    Y = 0.001
    R = 0.09
    D = 0.00203 # shank diameter of the needle
    E = 1000 #placeholder value
    p = 450 # placeholder value
    INSIDE_FRAME = 0.18*0.25
    h = 0.001 # fabric thickness
    a = np.radians(10) 
    cone_h = 0.0064
    P_base = 0.1 # unmeasured placeholder
    k = 6.6e7 # unmeasured placeholder


    tip = sim.site_pos("needle_tip")
    n = Fm[:, 2]
    p_s = p0 + 0.0005 * n
    d = tip - p0
    half = model.geom_size[fid][:2]          # (0.12, 0.085)
    inside_frame = abs(np.dot(d, Fm[:, 0])) <= half[0] and abs(np.dot(d, Fm[:, 1])) <= half[1]

    
    F = 0.47 # tangens of needle angle against fabric when it starts to slip (friction)
    P = 0.0
    
    if state["contact"] is None:
           if inside_frame and np.dot(tip - (p0 - h/2 * n), n) >= 0:
            state["contact"] = tip.copy()
           else:
            return 0.0, n, tip, 0.0
    
    L = np.dot(tip - state['contact'], n)
    dis = L - Y
    if 0 < L <= Y and not state["pierced"]: 
        P = np.pi * p * L + L**3 * (E *(V**3))/R**2
    elif Y < L <= Y + h:
        startcone, endcone = 0, dis
        fn, fr, fc, frc = friction_vals(endcone, startcone, 0)
        P = np.pi*p*Y + Y**3*E*V**3/R**2 + np.sin(a)*fn + np.cos(a)*fr
    elif (Y + h) < L <= (Y + cone_h):
        startcone = L - (Y + h)
        endcone = dis
        fn, fr, fc, frc = friction_vals(endcone, startcone, 0)
        P = P_base + np.sin(a) * fn + np.cos(a) * fr
        state["pierced"] = True
    elif (Y + cone_h) < L <= (Y + h + cone_h):
        startcone = L -(Y + h)
        endcone = cone_h
        cone_status = L - cone_h - Y
        fn, fr, fc, frc = friction_vals(endcone, startcone, cone_status)
        P = P_base + np.sin(a) * fn + np.cos(a) * fr + frc
    elif L > (Y + h + cone_h):
        cone_status = h
        fn, fr, fc, frc = friction_vals(0, 0, cone_status)
        P = P_base + frc
    return P, n, tip, L