import numpy as np
import construct_MP as cmp
import design_controller2 as K_des
import matplotlib as plt
from MC_construct import call_MC
# from figure_plot import probability_plot
import time
import json

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

def initialize2(vcar, vmax):
    Ncar = int(vmax*(vmax+1)/2 + 5)
    Vlow=  0
    Vhigh = vmax
    
    xcross_start = 2
    Nped = Ncar - xcross_start + 1
    xped = np.random.randint(1, Nped)
    
    # Choosing vmax:
    xped = np.random.randint(1, Nped)
    x_vcar_stop = vcar*(vcar+1)/2 + 1
    xcar_stop = (xped + xcross_start - 1)-1 # stop cell
    if x_vcar_stop <= xcar_stop:
        flg = 0 # These parameters can be used
        assert(xcar_stop > 0)
        state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1)+v
        bad_states = set()
        for vi in range(1,Vhigh+1):
            state = state_f(xcar_stop, vi)
            bad_states |= {"S"+str(state)}
        good_states= {"S" + str(state_f(xcar_stop,0))}
        bad = ""# Expression for bad states
        good = "" # Expression for good states
        for st in list(good_states):
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
    else:
        flg = 1 # Bad state
        Ncar = None
        Vlow = None
        Vhigh = None
        xcross_start = None
        xped = None
        bad_states = []
        good_states = []
        formula = ''
    return flg, Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_states, formula

# Initialize the velocity for vmax1:
ex= 2
if ex == 1:
    VMAX = []
    INIT_V = dict()
    P = dict()
    for vmax in range(1,6):
        INIT_V[vmax] = []
        P[vmax] = []
        print("===========================================================")
        print("Max Velocity: ", vmax)
        # Initial conditions set for all velocities
        Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula = initialize(vmax)
        print("Specification: ")
        print(formula)
        for vcar in range(1, vmax+1):  # Initial speed at starting point
            state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
            start_state = "S"+str(state_f(1,vcar))
            S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
            C = cmp.confusion_matrix_ped()
            K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped, vcar, xcross_start)
            true_env = str(xped) #Sidewalk 3
            true_env_type = "ped"
            O = {"ped", "obj", "empty"}
            state_info = dict()
            state_info["start"] = start_state
            state_info["bad"] = bad_states
            state_info["good"] = good_state
            M = call_MC(S, O, state_to_S, K, K_backup, C, true_env, true_env_type, state_info)
            result = M.prob_TL(formula)

            # Store results:
            VMAX.append(vmax)
            INIT_V[vmax].append(vcar)
            p = result[start_state]
            print('Probability of satisfaction for initial speed, {}, and max speed, {} is p = {}:'.format(vcar, vmax, p))
            P[vmax].append(result[start_state])
if ex == 2:
    VMAX = []
    INIT_V = dict()
    P = dict()
    # Initialize the velocity for where max velocity is not feasible but slower velocity is possible:
    for vmax in range(1,6):
        INIT_V[vmax] = []
        P[vmax] = []
        print("===========================================================")
        print("Max Velocity: ", vmax)
        # Initial conditions set for all velocities
        for vcar in range(1, vmax+1):  # Initial speed at starting point
            flg, Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula = initialize2(vcar, vmax)    
            if not flg:
                print("Specification: ")
                print(formula)
                state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
                start_state = "S"+str(state_f(1,vcar))
                S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
                C = cmp.confusion_matrix_ped()
                K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped, vcar, xcross_start)
                print("Correctly constructed controllers: ")
                true_env = str(xped) #Sidewalk 3
                true_env_type = "ped"
                O = {"ped", "obj", "empty"}
                state_info = dict()
                state_info["start"] = start_state
                state_info["bad"] = bad_states
                state_info["good"] = good_state
                M = call_MC(S, O, state_to_S, K, K_backup, C, true_env, true_env_type, state_info)
                # Evaluate formula:
                result = M.prob_TL(formula)
                VMAX.append(vmax)
                INIT_V[vmax].append(vcar)
                p = result[start_state]
                print('Probability of satisfaction from start state {} for initial speed, {}, and max speed, {} is p = {}:'.format(start_state, vcar, vmax, p))
                P[vmax].append(result[start_state])
            else:
                break
        # Evaluate formula:
        #    result = M.prob_TL(formula)

# Plotting algorithms:
# probability_plot(VMAX, INIT_V, P)
# Write to json file:
timestr = time.strftime("%Y%m%d-%H%M%S")
fname_v = "type_"+str(ex)+"_"+"init_v_" + timestr+"_.json"
fname_p = "type_"+str(ex)+"_"+"prob_" + timestr+"_.json"
with open(fname_v, 'w') as f:
    json.dump(INIT_V, f)
with open(fname_p, 'w') as f:
    json.dump(P, f)


