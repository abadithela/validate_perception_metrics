#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 09:12:22 2021

@author: apurvabadithela
"""
import numpy as np
from tulip.transys import MarkovChain as MC
from tulip.transys import MarkovDecisionProcess as MDP
from itertools import compress, product
from tulip.interfaces import stormpy as stormpy_int
import os
import ped_controller
import empty_controller
import not_ped_controller
from tulip.transys.compositions import synchronous_parallel

# Function to return a list of all combinations of inputs:
# Input: A dictionary names D
# Each key of D corresponds to a set of values that key can take
# Output: All combinations of input keys
def dict_combinations(D):
    keys = D.keys()
    values = list(D.values())
    prod_input = list(product(*values))
    return prod_input

# Function to calculate backup_controller:
def construct_backup_controller(Ncar, Vlow, Vhigh):
    K_backup = dict()
    for xcar in range(1,Ncar+1):
        for vcar in range(Vlow, Vhigh+1):
            st = (xcar, vcar)
            end_st = []
            if xcar == Ncar:
                end_st.append((xcar, vcar))
            elif vcar == 0:
                end_st.append((xcar, vcar))
                end_st.append((xcar+1, vcar+1))
                end_st.append((xcar, vcar+1))
            else:
                end_st.append((xcar+Vhigh, vcar))
                end_st.append((xcar+Vhigh, vcar-1))
            K_backup[st] = end_st
    return K_backup
# Script for confusion matrix of pedestrian
# Make this cleaner; more versatile
# C is a dicitionary: C(["ped", "nped"]) = N(observation|= "ped" | true_obj |= "nped") (cardinality of observations given as pedestrians while the true state is not a pedestrian)
def confusion_matrix_ped():
    C = dict()
    C["ped", "ped"] = 10/15
    C["ped", "obj"] = 2/15
    C["ped", "empty"] = 3/15
    
    C["obj", "ped"] = 2/15
    C["obj", "obj"] = 11/15
    C["obj", "empty"] = 2/15
    
    C["empty", "ped"] = 3/15
    C["empty", "obj"] = 3/15
    C["empty", "empty"] = 10/15
    
    return C

# Function that converts to a Markov chain from states and actions:
def _construct_mdpmc(states, transitions, init, actions=None):
    if actions is not None:
        ts = MDP()
        ts.actions.add_from(actions)
    else:
        ts = MC()
    ts.states.add_from(states)
    ts.states.initial.add(init)

    for transition in transitions:
        attr = {"probability": transition[2]}
        if len(transition) > 3:
            attr["action"] = transition[3]
        ts.transitions.add(
            transition[0],
            transition[1],
            attr,
        )

    for s in states:
        ts.atomic_propositions.add(s)
        ts.states[s]["ap"] = {s}

    return ts

# Creating the states of the markov chain for the system:
# Returns product states S and (pos,vel) to state dictionary
def system_states_example_ped(Ncar, Vlow, Vhigh):
    nS = Ncar*(Vhigh-Vlow+1)
    state = lambda x,v: (Vhigh-Vlow+1)*(x-1) + v
    state_to_S = dict()
    S = set()
    for xcar in range(1,Ncar+1):
        for vcar in range(Vlow, Vhigh+1):
            st = "S"+str(state(xcar, vcar))
            state_to_S[xcar,vcar] = st
            S|={st}
    K_backup = construct_backup_controller(Ncar, Vlow, Vhigh)
    return S, state_to_S, K_backup

# This script automatically generates a Markov process for modeling the probability
# of satisfaction of a temporal formula.
class synth_markov_chain:
    def __init__(self, S, O, state_to_S):
        self.states = S    # Product states for car.
        self.state_dict = state_to_S
        self.reverse_state_dict = {v: k for k, v in state_to_S.items()}
        self.obs = O
        self.true_env = None # This state is defined in terms of the observation
        self.true_env_type = None # Type of the env object; is in one of obs
        self.C = dict() # Confusion matrix dictionary giving: C[obs, true] =  P(obs |- phi | true |- phi)
        self.M = dict() # Two-by-two dictionary. a(i,j) = Prob of transitioning from state i to state j
        self.K = None # Depending on the observation, the controller changes. This is a dictionary of the controller after the Mealy machine is syntehsized from specifications 
        self.K_strategy = None # Dictionary containing the scripts to the controller after it has been written to file
        self.formula = []
        self.K_int_state_map = dict() # Nested dictionary mapping internal states to concrete states of the controller instantiation
        self.K_int_state_map_inv = dict() # Nested dictionary mapping concrete states into internal states
        self.MC = None # A Tulip Markov chain object that is consistent with TuLiP transition system markov chain
        self.true_env_MC = None # A Markov chain representing the true evolution of the environment
        self.backup = dict() # This is a backup controller.

 # Convert this Markov chain object into a tulip transition system:
    def to_MC(self, init):
       states = set(self.states) # Set of product states of the car
       transitions = set()
       for k in self.M.keys():
           t = (k[0], k[1], self.M[k])
           transitions |= {t}
       assert init in self.states
       # for state in mc.states:
       #    mc.states[state]["ap"] == {state}
       self.MC = _construct_mdpmc(states, transitions, init)
       markov_chain = _construct_mdpmc(states, transitions, init)
       for state in self.MC.states:
           self.MC.states[state]["ap"] = {state}
       self.check_MC() # Checking if Markov chain is valid
       return markov_chain
   
   # Function to check if all outgoing transition probabilities for any state in the Markov chain sum to 1.
    def check_MC(self):
       T = self.MC.transitions(data=True)
       # Printing all states and outgoing transitions
       for st in self.MC.states:
           print("State: ", st)
           end_states = [t for t in T if t[0]==st]
           print("End states: ")
           print(end_states)
           prob = [(t[2])['probability'] for t in end_states]
           print("probability of end states: ")
           print(prob)
           assert abs(sum(prob)-1)<1e-4 # Checking that probabilities add up to 1 for every state

   # Sets the state of the true environment
    def set_true_env_state(self, st, true_env_type):
        self.true_env = st
        new_st = "p"+st
        states = {new_st}
        transitions ={(new_st, new_st, 1)}
        init = new_st
        self.true_env_type = true_env_type
        self.true_env_MC = _construct_mdpmc(states, transitions, init)
     
    def set_confusion_matrix(self, C):
        self.C = C
    # Depending on the observation, the controller K changes. K should be a dictionary that maps observation to a controller. ToDo: Assess difference between K and K_strategy 
    def set_controller(self, K, K_strategy, K_backup):
        self.K_strategy = K_strategy
        self.K = K
        self.backup = K_backup

    def compute_next_state(self,obs,env_st, init_st): # The next state computed using the observation from the current state
        Ki = self.K[obs] # The controller object
        Ki_strategy =self.K_strategy[obs]
        K_instant, sinit_adj, flg = self.adjust_state(Ki, Ki_strategy,obs,init_st) # If flg = 1, use sinit_adj as given by backup controller
        if flg == 0:
            next_st = K_instant.move(*env_st)  # Might have to modify this when there are multiple observations
        else:
            next_st = {'xcar': sinit_adj[0], 'vcar': sinit_adj[1]}
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
            K_inst = Ki_strategy.TulipStrategy() # Instantiation
            N_int_states = K_inst.state # Total no. of internal states
            # print(inputs)
            # print(input_comb)
            # print("Inputs: ")
            # print(input_comb)
            # print("Outputs: ")
            # print(output_comb)
            # Check --> good upto this point
            self.K_int_state_map[Ki] = dict()
            self.K_int_state_map_inv[Ki] = dict()
            # Populating the dictionary:
            for ii in range(N_int_states):
                self.K_int_state_map[Ki][ii] = None # None object
            
            # Modifying the list:
            for int_st in range(N_int_states+1):
                # print("Internal state: ", str(int_st))
                for inp in input_comb:
                    K_inst.state = int_st # Modifying internal state to stay where it is
                    # print("Input: ", str(inp))
                    try: # Adding the environment state
                        out = K_inst.move(*inp)
                        state = K_inst.state
                        # print("State moved: ", str(state))
                        if (state == 2):
                            flg = 1
                        if self.K_int_state_map[Ki][state] is None:
                            # print("Output value: ", list(out.values()))
                            self.K_int_state_map[Ki][state] = list(out.values()) # Updating state
                            assert self.K_int_state_map[Ki][state] is not None
                    except Exception:
                        pass
            
            # Reversing the internal state map:
            self.K_int_state_map_inv[Ki] = {tuple(v): k for k,v in self.K_int_state_map[Ki].items()}
                
            
    # Adjust state of the system based on the controller that's being used so it doesn't throw an error:
    # Ki is the controller and Ki_strategy is the script that corresponds to the strategy
    # of Ki. One of the variables returned is also K_instant (the instantiation of the controller)
    def adjust_state(self,Ki, Ki_strategy, obs,  sinit_st):
        flg =0 # Default
        if sinit_st not in self.K_int_state_map_inv[Ki].keys():
            poss_st = self.backup[sinit_st]
            test_list = [p for p in poss_st if p in self.K_int_state_map_inv[Ki].keys()]
            if test_list: 
                sinit_st = test_list[0] # First state in the dictionary; deterministic controller
                init_st_adj=(self.K_int_state_map_inv[Ki])[sinit_st]
                K_instant = Ki_strategy.TulipStrategy()
                K_instant.state = init_st_adj
            else:
                flg = 1
                K_instant = None
                init_st_adj = poss_st[0]
        else:
            init_st_adj=(self.K_int_state_map_inv[Ki])[sinit_st] # Finding the mapping from internal states
            print(init_st_adj)
            K_instant = Ki_strategy.TulipStrategy()
            K_instant.state = init_st_adj   
        return K_instant, init_st_adj, flg
    
    # Function to return the state of the environment given the observation:
    def get_env_state(self, obs):
        env_st =[1] # For static environment, env state is the same. Should modify this function for reactive environments
        if obs == self.true_env_type:
            env_st = [int(self.true_env)]
        return env_st
    # Constructing the Markov chain 
    def construct_markov_chain(self): # Construct probabilities and transitions in the markov chain given the controller and confusion matrix
        for Si in list(self.states):
            init_st = self.reverse_state_dict[Si]
            # The output state can be different depending on the observation as defined by the confusion matrix
            for obs in self.obs:
                env_st = self.get_env_state(obs)
                next_st = self.compute_next_state(obs, env_st, init_st)
                print(next_st)
                print(self.state_dict)
                Sj = self.state_dict[tuple(next_st.values())]
                prob_t = self.C[obs, self.true_env_type] # Probability of transitions
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
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        prism_file_path = os.path.join(model_path, "pedestrian.nm")
        composed = synchronous_parallel([self.MC, self.true_env_MC])
        print(composed.transitions)
        result = stormpy_int.model_checking(composed, phi, prism_file_path)
        # result = stormpy_int.model_checking(self.MC, phi, prism_file_path) # Since there is no moving obstacle, try checking only the pedestrian obstacle
        # Debugging: print states of result:
        print(result)
        print(" ")
        for state in self.MC.states:
           print("  State {}, with labels {}, Pr = {}".format(state, self.MC.states[state], result[(state, None)]))
        # for state in self.MC.states:
        #   print("  State {}, with labels {}, Pr = {}".format(state, self.MC.states[state]["ap"], result[state]))
        return result # Implement this soon: Storm py
        
                    
                    
                
                
      
                
                
                
                
                
                
                
    
