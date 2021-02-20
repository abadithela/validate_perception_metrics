#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 20:22:34 2021

@author: apurvabadithela
"""

from __future__ import print_function

import logging

import numpy as np
from tulip import spec, synth, hybrid, transys
from polytope import box2poly
from tulip.abstract import prop2part, discretize
from tulip.abstract.plot import plot_partition
from tulip.dumpsmach import write_python_case

# This script invokes TuLiP to construct a controller for the system with respect to a 
# temporal logic specification based on the observed outputs of the perception algorithm
# Inputs to the controller synthesis function are: discrete_dynamics (disc_dynamics),
# cell/set of env. variables (env_vars), cell/set of system variables (sys_vars),
# cell/set of initial env. variables (env_init), cell/set of initial system variables (sys_init),
# enviornment and system safety and progress specifications: env_safe/env_prog and sys_safe/sys_prog.

def design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog):
    logging.basicConfig(level=logging.WARNING)
    show = False
    
    # Constructing GR1spec from environment and systems specifications:
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)
    specs.moore = True
    specs.qinit = '\E \A'
    
    # Synthesize
    ctrl = synth.synthesize(specs)
    assert ctrl is not None, 'unrealizable'
    
    return ctrl

# Function constructing transition system, specification variables for pedestrian/car example:
# Takes as input the geometry of the sidewalk:
# Ncar: No. of cells of the car, Nped: No. of cells of the crosswalk, xped: initial cell number of pedestrian, xcar: initial cell of car, vcar: initial velocity of car, xcross_start: Cell number of road at which the crosswalk starts
# Vlow: Lower integer speed bound for car, Vhigh: Upper integer speed bound for car
def not_pedestrianK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start):
    sys_vars = {}
    sys_vars['xcar'] = (1, Ncar)
    sys_vars['vcar'] = (Vlow, Vhigh)
    env_vars = {}
    env_vars['x_notped'] = (0,1) # 0 means no pedestrian
    
    sys_init = {'xcar='+str(xcar), 'vcar='+str(vcar)}
    env_init = {'x_notped='+str(1)}    
    
    sys_prog = set() # For now, no need to have progress
    env_prog = set()
    
    sys_safe = set()
    env_safe = set()
    
    # Add system dynamics to safety specs:
    for ii in range(1, Ncar+1):
        for vi in range(Vlow, Vhigh+1):
            if vi==0:
                spec_ii = {'((xcar='+str(ii)+') && (vcar=0))-> X((vcar=0 || vcar=1) && xcar='+str(ii)+')'}
                sys_safe|=spec_ii
            elif vi == Vhigh:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
            else:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+'|| vcar='+str(vi+1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog

# Controller for not_pedestrian observation:
def pedestrianK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start):
    sys_vars = {}
    sys_vars['xcar'] = (1, Ncar)
    sys_vars['vcar'] = (Vlow, Vhigh)
    env_vars = {}
    env_vars['xped'] = (1, Nped) # 0 means no pedestrian
    # env_vars['y_cr'] = (0,2) # 0: ped, 1: not_ped, 2: empty
    
    sys_init = {'xcar='+str(xcar), 'vcar='+str(vcar)}
    env_init = {'xped='+str(xped)}    
    
    sys_prog = set() # For now, no need to have progress
    env_prog = set()
    
    sys_safe = set()
    env_safe = set()
    
    # Environment safety specs: Static pedestrian
    env_safe |= {'xped='+str(xped)+'-> X(xped='+str(xped)+')'} 

    
    # system safety specs
    for ii in range(Nped, 0, -1):
        xcar_jj = ii+(xcross_start-1)-1
        assert xcar_jj > 0
        spec1_ii = {'(xcar='+str(xcar_jj)+') && (xped='+str(ii)+')-> (vcar=0)'}
        sys_safe |= spec1_ii
        spec2_ii = {'(xcar='+str(xcar_jj)+') && !(xped='+str(ii)+')-> (!(vcar=0))'}
        sys_safe |= spec2_ii
    
    # Add system dynamics to safety specs:
    for ii in range(1, Ncar+1):
        for vi in range(Vlow, Vhigh+1):
            if vi==0:
                spec_ii = {'((xcar='+str(ii)+') && (vcar=0))-> X((vcar=0 || vcar=1) && xcar='+str(ii)+')'}
                sys_safe|=spec_ii
            elif vi == Vhigh:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
            else:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+'|| vcar='+str(vi+1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog

# Controller for empty observation:
def emptyK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start):
    sys_vars = {}
    sys_vars['xcar'] = (1, Ncar)
    sys_vars['vcar'] = (Vlow, Vhigh)
    env_vars = {}
    env_vars['xempty'] = (0,1) # 0: pavement is not empty, 1: pavement is empty
    
    sys_init = {'xcar='+str(xcar), 'vcar='+str(vcar)}
    env_init = {'xempty='+str(1)}    
    
    sys_prog = set() # For now, no need to have progress
    env_prog = set()
    
    sys_safe = set()
    env_safe = set()
    
    # Environment safety specs: Static pedestrian
    # env_safe |= {'xped='+str(xped)+'-> X(xped='+str(xped)+')'} 

    # Add system dynamics to safety specs:
    for ii in range(1, Ncar+1):
        for vi in range(Vlow, Vhigh+1):
            if vi==0:
                spec_ii = {'((xcar='+str(ii)+') && (vcar=0))-> X((vcar=0 || vcar=1) && xcar='+str(ii)+')'}
                sys_safe|=spec_ii
            elif vi == Vhigh:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
            else:
                xf_ii = min(ii+vi, Ncar+1)
                spec_ii = {'((xcar='+str(ii)+') && (vcar='+str(vi)+'))-> X((vcar='+str(vi)+'|| vcar='+str(vi-1)+'|| vcar='+str(vi+1)+') && xcar='+str(xf_ii)+')'}
                sys_safe|=spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog


if __name__=='__main__':
    # Simple example of pedestrian crossing street:
    Ncar = 5
    Nped = 3
    Vhigh = 1
    Vlow = 0
    xcar = 1
    vcar = Vhigh
    xped = 3
    xcross_start = 3
    assert (Nped + xcross_start-1 <= Ncar)
    # When a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = pedestrianK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start)
    Kped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("ped_controller.py", Kped)
    
    # When something other than a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = not_pedestrianK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start)
    Knot_ped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("not_ped_controller.py", Knot_ped)
    
    # When nothing is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = emptyK(Ncar, Nped, xcar, vcar, Vlow, Vhigh, xped, xcross_start)
    Kempty = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("empty_controller.py", Kempty)