#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 11:26:03 2021

@author: apurvabadithela
"""

import numpy as np
import tulip
from ped_ontroller import TulipStrategy

C = TulipStrategy()
C.state = 3
obs = [3]
out = C.move(*obs)
print(out["xcar"])
print(out["vcar"])