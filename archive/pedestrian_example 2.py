import numpy as np
import construct_MP as cmp
import design_controller as K_des
# import ped_controller as Kped
# import not_ped_controller as Kobj
# import empty_controller as Kempty
import matplotlib as plt

from test_ped import initialize

for vmax in range(1,3):
    Ncar, Vlow, Vhigh, xcross_start, xped, bad_states, good_state, formula = initialize(vmax)
    for vcar in range(1, vmax+1):
        state_f = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
        start_state = "S"+str(state_f(1,vcar))
        S, state_to_S, K_backup = cmp.system_states_example_ped(Ncar, Vlow, Vhigh)
        C = cmp.confusion_matrix_ped()
        K = K_des.construct_controllers(Ncar, Vlow, Vhigh, xped, vcar, xcross_start)
        true_env = str(xped) #Sidewalk 3
        true_env_type = "ped"
        O = {"ped", "obj", "empty"}
        K_strat = dict()
        
        import ped_controller as Kped
        import not_ped_controller as Kobj
        import empty_controller as Kempty

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
        MC = M.to_MC(start_state, bad_states, good_state) # For setting initial conditions and assigning bad/good labels
        print(M.M)

    # Evaluate formula:
        result = M.prob_TL(formula)

