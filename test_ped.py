import numpy as np
import construct_MP as cmp
import design_controller as K_des
import ped_controller as Kped
import not_ped_controller as Kobj
import empty_controller as Kempty

# Initializing parameters for function:
Ncar = 5
Vlow = 0
Vhigh = 1

S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
C = cmp.confusion_matrix_ped()
K = K_des.construct_controllers(Ncar, Vlow, Vhigh)
true_env = "3"#Sidewalk 3
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

# Add formula:
formula = 'P=? [G(!("S7" & "p3"))]'
formula2 = 'P=? [F("S6")]'# Eventually reach a good state
formula3 = 'P=? [!("S7") U "S6"]'
#formula = 'P=? [G(!("S7"))]'# phi = []((xcar = 4 & ped = 3) --> (vcar = 0)): !("S7"), where S7: xcar = 4 and vcar  = 1

# Getting probability of satisfaction:
result = M.prob_TL(formula3)
