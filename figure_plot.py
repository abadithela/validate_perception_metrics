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
    plt.ylabel("Probability")
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
        axi.set_ylim(0, 1)
        axi.set_yticks(np.linspace(0,0.75,3))
    # for axi in ax.flat:
    #     axi.set(xlabel='Initial speed')
    fig.text(0.5, 0.01, 'Initial speed', ha='center')
    fig.text(0.01, 0.5, 'Probability', va='center', rotation='vertical')
    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for axi in ax.flat:
        axi.label_outer()
   
    plt.xticks(np.arange(1,11,1))
    plt.savefig(fig_name, format='png', dpi=1200)
    plt.savefig('destination_path.eps', format='eps')
    plt.show()

def probability_individual_plot2(INIT_V, P, fig_name):
    fig1, ax1 = plt.subplots(len(INIT_V)//2, 1)
    #plt.xticks(np.arange(1,11,1))
    fig2, ax2 = plt.subplots(len(INIT_V)//2, 1)
    for k,v in INIT_V.items():
        probabilities = P[k]
        init_speed = INIT_V[k]
        row = (int(k)-1)%5
        col = 0
        if int(k) <= 5:
            ax1[row].plot(init_speed, probabilities, 'o--', label=r"$V_{max}$"+f"={k}")
            leg = ax1[row].legend(loc="best")
        else:
            ax2[row].plot(init_speed, probabilities, 'o--', label=r"$V_{max}$"+f"={k}")
            leg = ax2[row].legend(loc="best")
    for axi in ax1.flat:
        axi.set_ylim(0, 1)
        axi.set_yticks(np.linspace(0,0.75,3))
        axi.set_xticks(np.linspace(1,row+1,row+1))
    for axi in ax2.flat:
        axi.set_ylim(0, 1)
        axi.set_yticks(np.linspace(0,0.75,3))
        axi.set_xticks(np.linspace(1,row+6,row+6))
    # for axi in ax.flat:
    #     axi.set(xlabel='Initial speed')
    fig1.text(0.5, 0.01, 'Initial speed', ha='center')
    fig1.text(0.01, 0.5, 'Probability', va='center', rotation='vertical')
    fig2.text(0.5, 0.01, 'Initial speed', ha='center')
    fig2.text(0.01, 0.5, 'Probability', va='center', rotation='vertical')
    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for axi in ax1.flat:
        axi.label_outer()
    for axi in ax2.flat:
        axi.label_outer()
    
    #plt.xticks(np.arange(1,11,1))
    fig1.savefig(fig_name+"_p1.png", format='png', dpi=300)
    fig2.savefig(fig_name+"_p2.png", format='png', dpi=300)
    plt.show()
    
fname_v = "type_1_init_v_20210228-184123_.json"
fname_p = "type_1_prob_20210228-184123_.json"
fname_v = "type_1_same_MC_init_v_20210301-192151_.json"
fname_p = "type_1_same_MC_prob_20210301-192151_.json"
fname_v = "test_type1_vmax_10_initv.json"
fname_p = "test_type1_vmax_10_prob.json"

with open(fname_v) as fv:
    INIT_V = json.load(fv)
with open(fname_p) as fp:
    P = json.load(fp)
fig_name = "type_1_prob_20210309"
# probability_plot(INIT_V, P, fig_name)
probability_individual_plot2(INIT_V, P, fig_name)
# probability_split_plot(INIT_V, P, fig_name)
# plt.savefig('destination_path.eps', format='eps')
