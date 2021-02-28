import numpy as np
import construct_MP as cmp
import design_controller as K_des
import ped_controller as Kped
import not_ped_controller as Kobj
import empty_controller as Kempty
import matplotlib as plt

# Initializing parameters for function:
vmax = 3
if vmax == 3:
    Ncar = 7
    Vlow = 0
    Vhigh = 3
    xped = 3
    xcross_start = 5
    vcar = Vhigh//2
    good_states = {"S20"}
    bad_states = {"S21", "S22", "S23"}
    good = ""
    bad = ""
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

    phi = "P=? [!("+good+") U ("+bad+")]"

if vmax == 2:
    Ncar = 7
    Vlow = 0
    Vhigh = 2
    xped = 3
    vcar = Vhigh
    xcross_start = 4
    phi = 'P=? [!("S13" | "S14")U "S12"]'

if vmax == 1:
    Ncar = 1
    Vlow = 0
    Vhigh = 1
    xped = 3
    xcross_start = 3
    vcar = Vhigh
    phi = 'P=? [!("S7")U"S6"]'

S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
C = cmp.confusion_matrix_ped()
K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped, vcar, xcross_start)
true_env = str(xped) #Sidewalk 3
true_env_type = "ped"
O = {"ped", "obj", "empty"}
K_strat = dict()
K_strat["ped"] = Kped
K_strat["obj"] = Kobj
K_strat["empty"] = Kempty
M = cmp.synth_markov_chain(S, O, state_to_S)
M.set_confusion_matrix(C)
M.set_true_env_state(true_env, true_env_type)
M.set_controller(K, K_strat, K_backup)

M.construct_internal_state_maps()

# Construct Markov chain:
M.construct_markov_chain()
start_state="S1"
good_states = {"S20"}
bad_states = {"S21", "S22", "S23"}
MC = M.to_MC(start_state, bad_states, good_states) # For setting initial conditions and assigning bad/good labels
print(M.M)
result = M.prob_TL(phi)
M.print_MC()

print(result)
print("Find state: ")
print(result[start_state])
print("Input dictionary: ")

# def initialize(vmax):
#    Ncar = 10
#    Vlow=  0
#    Vhigh = vmax
#    x_vmax_stop = vmax*(vmax+1)/2
#    xcross_start = 2
#    Nped = Ncar - xcross_start + 1
#    min_xped = x_vmax_stop + 1 - (xcross_start - 1)
#    assert(min_xped > 0)
#    xped = np.random.randint(min_xped, Nped)
#    xped = min_xped
#    xcar_stop = xped + xcross_start - 2
#    assert(xcar_stop > 0)
#    state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1)+v
#    bad_states = set()
#    for vi in range(1,Vhigh+1):
#        state = state_f(xcar_stop, vi)
#        bad_states |= {"S"+str(state)}
#    good_state = {"S" + str(state_f(xcar_stop,0))}
#    formula = 'P=? [!("bad") U "good"]'
#    return Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula

#for vmax in range(1,3):
#    Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula = initialize(vmax)
#    for vcar in range(1, vmax+1):
#        state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
#        start_state = "S"+str(state_f(1,vcar))
#        S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
#        C = cmp.confusion_matrix_ped()
#        K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped, vcar, xcross_start)
#        true_env = str(xped) #Sidewalk 3
#        true_env_type = "ped"
#        O = {"ped", "obj", "empty"}
#        K_strat = dict()
#        K_strat["ped"] = Kped
#        K_strat["obj"] = Kobj
#        K_strat["empty"] = Kempty

#        M = cmp.synth_markov_chain(S, O, state_to_S)
#        M.set_confusion_matrix(C)
#        M.set_true_env_state(true_env, true_env_type)
#        M.set_controller(K, K_strat, K_backup)

#        M.construct_internal_state_maps()

    # Construct Markov chain:
#        M.construct_markov_chain()
#        MC = M.to_MC(start_state, bad_states, good_state) # For setting initial conditions and assigning bad/good labels
#        print(M.M)
    
    # Evaluate formula:
#        result = M.prob_TL(formula)

    # Store data:
        
# Add formula:
# xcar_des = (xcross_start-1) + xped - 1
# vcar_des = 0
# reach_state = (Vhigh-Vlow+1)*(xcar_des-1) + vcar_des
# formula = 'P=? [G(!("S7" & "p3"))]'
# formula2 = 'P=? [F("S18")]'# Eventually reach a good state
# formula3 = 'P=? [!("S10") U "S9"]'
#formula = 'P=? [G(!("S7"))]'# phi = []((xcar = 4 & ped = 3) --> (vcar = 0)): !("S7"), where S7: xcar = 4 and vcar  = 1

# Getting probability of satisfaction:
# result = M.prob_TL(phi)
