import matplotlib.pyplot as plt
import numpy as np
import json

def probability_plot(INIT_V, P):
    fig, ax = plt.subplots()
    for k,v in INIT_V.items():
        probabilities = P[k]
        init_speed = INIT_V[k]
        plt.plot(probabilities, init_speed, 'o--', label=f"V_{max}={k}")
    leg = plt.legend(loc="best")
    plt.show()
    
fname_v = "type_1_init_v_20210228-045604_.json"
fname_p = "type_1_prob_20210228-045604_.json"

with open(fname_v) as fv:
    INIT_V = json.load(fv)
with open(fname_p) as fp:
    P = json.load(fp)

probability_plot(INIT_V, P)