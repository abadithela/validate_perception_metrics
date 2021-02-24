import os
from tulip.interfaces import stormpy as stormpy_int
from tulip.transys.compositions import synchronous_parallel

model_path = os.path.join("/home/ubuntu/eeci/c1-probabilistic", "models")
ma_path = os.path.join(model_path, "ma.nm")
mh_path = os.path.join(model_path, "mh.pm")
light_path = os.path.join(model_path, "light.pm")

ma = stormpy_int.to_tulip_transys(ma_path)
mh = stormpy_int.to_tulip_transys(mh_path)
light = stormpy_int.to_tulip_transys(light_path)

composed = synchronous_parallel([mh, light])

formula = 'P=? [ "green" U "h6" ]'

out_model_path = os.path.join(model_path, "out_composed_model.nm")
result = stormpy_int.model_checking(composed, formula, out_model_path)

for state in composed.states:
    print("  State {}, with labels {}, Pr = {}".format(state, composed.states[state]["ap"], result[state]))
    
