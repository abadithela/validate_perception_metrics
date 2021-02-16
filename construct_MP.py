#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 09:12:22 2021

@author: apurvabadithela
"""
import numpy as np
from itertools import compress, product

# Function to return a list of all combinations of inputs:
# Input: A dictionary names D
# Each key of D corresponds to a set of values that key can take
# Output: All combinations of input keys
def dict_combinations(D):
    keys = D.keys()
    values = list(D.values())
    prod_input = list(product(*values))
    return prod_input

# Script for confusion matrix of pedestrian
# Make this cleaner; more versatile
# C is a dicitionary: C(["ped", "nped"]) = N(observation|= "ped" | true_obj |= "nped") (cardinality of observations given as pedestrians while the true state is not a pedestrian)
def confusion_matrix_ped():
    C = dict()
    C["ped", "ped"] = 10
    C["ped", "not_ped"] = 1
    C["ped", "empty"] = 3
    
    C["not_ped", "ped"] = 2
    C["not_ped", "not_ped"] = 11
    C["not_ped", "empty"] = 2
    
    C["empty", "ped"] = 3
    C["empty", "not_ped"] = 3
    C["empty", "empty"] = 10
    
    return C

# Creating the states of the markov chain for the system:
# Returns product states S and (pos,vel) to state dictionary
def system_states_example_ped(Ncar, Vlow, Vhigh):
    nS = Ncar*(Vhigh-Vlow)
    state = lambda x,v: x + (Vhigh-Vlow)*(v-1)
    state_to_S = dict()
    S = set()
    for xcar in range(1,Ncar+1):
        for vcar in range(Vlow, Vhigh+1):
            st = "S"+str(state(xcar, vcar))
            state_to_S[xcar,vcar] = st
            S|={st}
    return S, state_to_S

# This script automatically generates a Markov process for modeling the probability
# of satisfaction of a temporal formula.
class synth_markov_chain:
    def __init__(self, S, O, state_to_S):
        self.states = S    # Product states for car.
        self.state_dict = state_to_S
        self.reverse_state_dict = {v: k for k, v in state_to_S.items()}
        self.obs = O
        self.true_env = None # This state is defined in terms of the observation
        self.C = dict() # Confusion matrix dictionary giving: C[obs, true] =  P(obs |- phi | true |- phi)
        self.M = dict() # Two-by-two dictionary. a(i,j) = Prob of transitioning from state i to state j
        self.K = None # Depending on the observation, the controller changes
        self.K_strategy = None # Dictionary containing the scripts to the controller
        self.formula = []
        self.K_int_state_map = None # Nested dictionary mapping internal states to concrete states of the controller instantiation
        self.K_int_state_map_inv = None # Nested dictionary mapping concrete states into internal states
        
    # Sets the state of the true environment
    def set_true_env_state(self, st):
        self.true_env = st
    def set_confusion_matrix(self, C):
        self.C = C
    # Depending on the observation, the controller K changes. K should be a dictionary that maps observation to a controller
    # and uses the appropriate controller to 
    def set_controller(self, K):
        self.K = K
    def compute_next_state(self, K, K_strategy, obs, init_st): # The next state computed using the observation from the current state
        Ki = K[obs] # The controller object
        Ki_strategy = K_strategy[obs]
        K_instant = self.adjust_state(Ki, Ki_strategy, init_st)
        next_st = K_instant.move(*obs)  # Might have to modify this when there are multiple observations
        return next_st
    
    # Function to construct a map of internal states of various controllers:
    def construct_internal_state_maps(self):
        for obs in self.obs:
            Ki = self.K[obs]
            Ki_strategy = self.K_strategy[obs]
            inputs = Ki.inputs # Input dictionary mapping input variables to range of values
            outputs = Ki.outputs # Output dictionary mapping output variable to range of values
            input_comb = dict_combinations(inputs)
            output_comb = dict_combinations(outputs)
            K_inst = Ki_strategy() # Instantiation
            N_int_states = K_inst.state # Total no. of internal states
            
            self.K_int_state_map[Ki] = dict()
            self.K_int_state_map_inv[Ki] = dict()
            # Populating the dictionary:
            for ii in range(N_int_states):
                self.K_int_state_map[Ki][ii] = None # None object
            
            # Modifying the list:
            for int_st in range(N_int_states):
                K_inst.state = int_st # modifying the state
                for inp in input_comb:
                    try: # Adding the environment state
                        out = K_inst.move(*inp)
                        state = K_inst.state
                        if self.K_int_state_map[Ki][state] is None:
                            self.K_int_state_map[Ki][state] = list(out) # Updating state
                    except Exception:
                        pass
            
            # Reversing the internal state map:
            self.K_int_state_map_inv[Ki] = {v: k for k,v in self.K_int_state_map[Ki]}
                
            
    # Adjust state of the system based on the controller that's being used so it doesn't throw an error:
    # Ki is the controller and Ki_strategy is the script that corresponds to the strategy
    # of Ki. One of the variables returned is also K_instant (the instantiation of the controller)
    def adjust_state(self, Ki, Ki_strategy, sinit_st):
        init_st_adj=self.K_int_state_map_inv[Ki][sinit_st] # Finding the mapping from internal states
        K_instant = Ki_strategy()
        K_instant.state = init_st_adj
        return K_instant
    
    # Constructing the Markov chain 
    def construct_markov_chain(self): # Construct probabilities and transitions in the markov chain given the controller and confusion matrix
        for Si in self.states():
            init_st = self.reverse_state_dict[Si]
            # The output state can be different depending on the observation as defined by the confusion matrix
            for obs in self.obs:
                next_st = self.compute_next_state(self.K, obs, init_st)
                Sj = self.state_dict[next_st]
                prob_t = self.C[obs, self.true_env] # Probability of transitions
                if (Si, Sj) in self.M.keys():
                    self.M[Si, Sj] = self.M[Si, Sj] + prob_t
                else:
                    self.M[Si, Sj] = prob_t
        return self.M
    # Adding formulae to list of temporal logic formulas:
    def add_TL(self, phi):
        self.formula.append(phi)

    # Probabilistic satisfaction of a temporal logic with respect to a model:
    def prob_TL(self, phi):
        pass # Implement this soon: Storm py
        
                    
                    
                
                
                
                
                
                
                
                
                
                
                
            