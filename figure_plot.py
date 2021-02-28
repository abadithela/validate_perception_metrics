import matplotlib.pyplot as plt
import numpy as np
import json

def probability_plot(INIT_V, P, fig_name):
    fig, ax = plt.subplots()
    for k,v in INIT_V.items():
        probabilities = P[k]
        init_speed = INIT_V[k]
        plt.plot(init_speed, probabilities, 'o--', label=f"V={k}")
    leg = plt.legend(loc="best")
    plt.xlabel("Initial speed")
    plt.ylabel("Probability of satisfaction")
    plt.xticks(np.arange(1,10,1))
    plt.savefig(fig_name, format='png', dpi=1200)
    plt.show()
    
fname_v = "type_1_init_v_20210228-184123_.json"
fname_p = "type_1_prob_20210228-184123_.json"

with open(fname_v) as fv:
    INIT_V = json.load(fv)
with open(fname_p) as fp:
    P = json.load(fp)
fig_name = "type_1_init_v_20210228-184123_.png"
probability_plot(INIT_V, P, fig_name)
