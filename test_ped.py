import numpy as np
import construct_MP as cmp
import design_controller as K_des
import ped_controller as Kped
import not_ped_controller as Kobj
import empty_controller as Kempty

# Initializing parameters for function:
vmax = 3
if vmax == 3:
    Ncar = 7
    Vlow = 0
    Vhigh = 3
    xped = 3
    xcross_start = 5
    vcar = Vhigh//2
    phi = 'P=? [!("S23"|"S22" | "S21") U "S20"]'

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
# print(M.K_int_state_map)

# Construct Markov chain:
M.construct_markov_chain()
MC = M.to_MC("S1")
print(M.M)

# Add formula:
xcar_des = (xcross_start-1) + xped - 1
vcar_des = 0
reach_state = (Vhigh-Vlow+1)*(xcar_des-1) + vcar_des
formula = 'P=? [G(!("S7" & "p3"))]'
formula2 = 'P=? [F("S18")]'# Eventually reach a good state
formula3 = 'P=? [!("S10") U "S9"]'
#formula = 'P=? [G(!("S7"))]'# phi = []((xcar = 4 & ped = 3) --> (vcar = 0)): !("S7"), where S7: xcar = 4 and vcar  = 1

# Getting probability of satisfaction:
result = M.prob_TL(phi)
