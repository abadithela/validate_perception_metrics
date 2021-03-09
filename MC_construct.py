import ped_controller as Kped
import not_ped_controller as Kobj
import empty_controller as Kempty
import construct_MP3 as cmp
import importlib

def call_MC(S, O, state_to_S, K, K_backup, C, true_env, true_env_type, state_info):
    importlib.reload(Kped)
    importlib.reload(Kobj)
    importlib.reload(Kempty)
    
    K_strat = dict()
    K_strat["ped"] = Kped
    K_strat["obj"] = Kobj
    K_strat["empty"] = Kempty
    
    obs_keys = dict()
    obs_keys["ped"] = ["xcar", "vcar"]
    obs_keys["obj"] = ["xobj"]
    obs_keys["empty"] = ["xempty"]
    
    M = cmp.synth_markov_chain(S, O, state_to_S)
    M.set_confusion_matrix(C)
    M.set_true_env_state(true_env, true_env_type)
    M.set_controller(K, K_strat, K_backup)
    
    M.construct_internal_state_maps()
    
    # Construct Markov chain:
    M.construct_markov_chain()
    start_state = state_info["start"]
    bad_states = state_info["bad"]
    good_state = state_info["good"]
    MC = M.to_MC(start_state, bad_states, good_state) # For setting initial conditions and assigning bad/good labels
    print(M.M)
    return M
