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

def probability_split_plot(INIT_V, P, fig_name):
    fig, ax = plt.subplots(2)
    for k,v in INIT_V.items():
        probabilities = P[k]
        init_speed = INIT_V[k]
        if int(k)<=5:
            ax[0].plot(init_speed, probabilities, 'o--', label=f"V={k}")
        else:
            ax[1].plot(init_speed, probabilities, 'o--', label=f"V={k}")
    leg = ax[0].legend(loc="best")
    leg2 = ax[1].legend(loc="best")
    for axi in ax.flat:
        axi.set(xlabel='Initial speed', ylabel='Probability')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for axi in ax.flat:
        axi.label_outer()
    plt.xlabel("Initial speed")
    plt.ylabel("Probability of satisfaction")
    plt.xticks(np.arange(1,11,1))
    plt.savefig(fig_name, format='png', dpi=1200)
    plt.show()

def probability_individual_plot(INIT_V, P, fig_name):
    fig, ax = plt.subplots(len(INIT_V)//2, 2)
    for k,v in INIT_V.items():
        probabilities = P[k]
        init_speed = INIT_V[k]
        row = (int(k)-1)%5
        col = (int(k)-1)//5
        ax[row, col].plot(init_speed, probabilities, 'o--', label=f"V={k}")
        leg = ax[row, col].legend(loc="best")
    for axi in ax.flat:
        axi.set(xlabel='Initial speed', ylabel='Probability')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for axi in ax.flat:
        axi.label_outer()
   
    plt.xticks(np.arange(1,11,1))
    plt.savefig(fig_name, format='png', dpi=1200)
    plt.show()
    
fname_v = "type_1_init_v_20210228-184123_.json"
fname_p = "type_1_prob_20210228-184123_.json"
fname_v = "type_1_same_MC_init_v_20210301-192151_.json"
fname_p = "type_1_same_MC_prob_20210301-192151_.json"

with open(fname_v) as fv:
    INIT_V = json.load(fv)
with open(fname_p) as fp:
    P = json.load(fp)
fig_name = "type_1_same_MC_prob_20210301-192151_.png"
probability_individual_plot(INIT_V, P, fig_name)
