#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 11:08:00 2021
@author: apurvabadithela
"""

import numpy as np
import construct_MP3 as cmp
import design_controller4 as K_des
import matplotlib as plt
from MC_construct2 import call_MC
# from figure_plot import probability_plot
import time
import json
import sys
sys.setrecursionlimit(10000)

def initialize(vmax):
    Ncar = int(vmax*(vmax+1)/2 + 5)
    Vlow=  0
    Vhigh = vmax
    x_vmax_stop = vmax*(vmax+1)/2 + 1
    xcross_start = 2
    Nped = Ncar - xcross_start + 1
    if x_vmax_stop >= xcross_start:
        min_xped = int(x_vmax_stop + 1 - (xcross_start - 1))
    else:
        min_xped = 3
    assert(min_xped > 0)
    assert(min_xped<= Nped)
    if min_xped < Nped:
        xped = np.random.randint(min_xped, Nped)
    else:
        xped = int(min_xped)
    xped = int(min_xped)
    xcar_stop = xped + xcross_start - 2
    assert(xcar_stop > 0)
    state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1)+v
    bad_states = set()
    for vi in range(1,Vhigh+1):
        state = state_f(xcar_stop, vi)
        bad_states |= {"S"+str(state)}
    good_state = {"S" + str(state_f(xcar_stop,0))}
    bad = "" # Expression for bad states
    good = "" # Expression for good states
    for st in list(good_state):
        if good == "":
            good = good + "\"" + st+"\""
        else:
            good = good + "|\""+st+"\""
    for st in list(bad_states):
        if bad == "":
            bad = bad + "\"" + st+"\""
        else:
            bad = bad + "|\""+st+"\""
    formula = "P=?[!("+str(bad)+") U "+str(good)+"]"
    return Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula

ex= 1
if ex == 1:
    VMAX = []
    INIT_V = dict()
    P = dict()
    for vmax in range(1, 3):
        INIT_V[vmax] = dict()
        P[vmax] = dict()
        print("===========================================================")
        print("Max Velocity: ", vmax)
        # Initial conditions set for all velocities
        Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula = initialize(vmax)
        print("Specification: ")
        print(formula)

        count = 0
        for xped_i in range(xped, Nped+1):
            INIT_V[vmax][count] = []
            P[vmax][count]= []
            print(xped_i)
            Nped = Ncar - xcross_start + 1
            count = 0
            for vcar in range(1, vmax+1):  # Initial speed at starting point
                state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
                start_state = "S"+str(state_f(1,vcar))
                print(start_state)
                S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
                C = cmp.confusion_matrix_ped()
                K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped_i, vcar, xcross_start)
                Nped = Ncar - xcross_start+1
                true_env = str(count) #Sidewalk 3
                true_env_type = "ped"
                O = {"ped", "obj", "empty"}
                state_info = dict()
                state_info["start"] = start_state
                state_info["bad"] = bad_states
                state_info["good"] = good_state
                for st in list(good_state):
                    formula2 = 'P=?[F(\"'+st+'\")]'
                M= call_MC(S, O, state_to_S, K, K_backup, C, true_env, true_env_type, state_info)
                result = M.prob_TL(formula)
                result2 = M.prob_TL(formula2)
                print('Probability of eventually reaching good state for initial speed, {}, and max speed, {} is p = {}:'.format(vcar, vmax, result2[start_state]))
            # Store results:
            VMAX.append(vmax)
            INIT_V[vmax].append(vcar)
            p = result[start_state]
            print('Probability of satisfaction for initial speed, {}, and max speed, {} is p = {}:'.format(vcar, vmax, p))
            P[vmax].append(result[start_state])
# # Write to json file:
timestr = time.strftime("%Y%m%d-%H%M%S")
fname_v = "type_"+str(ex)+"_"+"init_v_" + timestr+"_.json"
fname_p = "type_"+str(ex)+"_"+"prob_" + timestr+"_.json"
fname_v = "test_type1_vmax_10_initv.json"
fname_p = "test_type1_vmax_10_prob.json"
with open(fname_v, 'w') as f:
    json.dump(INIT_V, f)
with open(fname_p, 'w') as f:
    json.dump(P, f)


