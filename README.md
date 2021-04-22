# Validate_perception_metrics

This branch contains code for reproducing the results in "Leveraging Classification Metrics for Quantitative System-Level Analysis" by Apurva Badithela, Tichakorn Wongpiromsarn, and Richard M. Murray submitted to CDC 2021. 
Specifically, this repo contains the code for reproducing the results for the following analyses:
1. Satisfaction probability of the system requirements for various environmental states for different initial conditions of the agent.
2. Satisfaction probability of the system requirements for various environmental states for various confusion matrices parametrized by precision and recall parameters.

This code was implemented on Ubuntu 18.04.

# Instructions:
1. All code to reproduce the results of the paper is in the master branch
2. The main files to run are the test_ped_7.py and test_ped_9.py.

# Main Requirements:
1. Storm (https://www.stormchecker.org) and Stormpy (https://moves-rwth.github.io/stormpy/)
2. TuLiP (https://github.com/tulip-control/tulip-control)

Instructions for installing Storm and Stormpy are listed below and can also be found here: https://murray.cds.caltech.edu/EECI-IGSC_2020

polytope: Make sure you have version 0.2.2 or higher of polytope installed
```bash
   $ python -c "import polytope; print(polytope.__version__)"
```
If the version is not '0.2.2' (possibly followed by some additional text, e.g., 0.2.2.dev0+f12c87a64641fed4d36a0fe904613495c434577d), then you need to install the latest version of polytope from source:
```bash   
   $ git clone https://github.com/tulip-control/polytope.git
   $ python setup.py install
```
matplotlib: matplotlib is not required for TuLiP but will be used in the course for visualization
```bash
   $ pip install matplotlib
```
dot: dot is not required for TuLiP but is used for visualization. The dot program is part of the graphviz package available on most *nix systems. A typical way to install the package is to use the following command
```bash
   $ sudo apt-get install graphviz
```
Stormpy: stormpy requires multiple packages, including carl, pycarl, z3 and storm. First, get all the required libraries. I summarize it here based on Ubuntu. (I tried it on Ubuntu18.04 but other versions should work too.)
```bash
   $ sudo snap install cmake --classic
   $ sudo apt install build-essential libgmp3-dev libeigen3-dev libboost-all-dev libcln-dev ginac-tools autoconf glpk-utils hwloc libginac-dev automake libglpk-dev libhwloc-dev libz3-dev libxerces-c-dev libeigen3-dev
```
carl:
```bash
   $ git clone https://github.com/smtrat/carl.git
   $ cd carl
   $ git checkout master14
   $ mkdir build && cd build
   $ cmake -DUSE_CLN_NUMBERS=ON -DUSE_GINAC=ON -DTHREAD_SAFE=ON ..
   $ make lib_carl
```
pycarl:
```bash
   $ git clone https://github.com/moves-rwth/pycarl.git
   $ cd pycarl/
   $ python setup.py develop
```
z3:
```bash
   $ git clone https://github.com/Z3Prover/z3.git
   $ cd z3
   $ python scripts/mk_make.py
   $ cd build
   $ make
   $ sudo make install
```
Note down where z3 is installed. If you use virtualenv, it should be something like venv_path/bin/z3 where venv_path is the path to the virtual environment.

Now, we procced to install Storm and Stormpy.
Important: Please ensure that you install the correct versions of Storm and Stormpy that are compatible with each other. For the purposes of this repo, we recommend installing the version 1.6.3 release of Storm (https://github.com/moves-rwth/storm/releases/tag/1.6.3) and Stormpy (https://github.com/moves-rwth/stormpy/releases/tag/1.6.3).
After downloading the source code for Storm and Stormpy, the follow these instructions for setting up the packages:
storm:
```
$ cd storm
$ export STORM_DIR=path_to_storm
$ mkdir build
$ cd build
$ cmake -DUSE_CLN_NUMBERS=ON -DUSE_GINAC=ON -DTHREAD_SAFE=ON ..
$ ccmake ..
 Change the followings:
   Z3_EXEC: venv_path/bin/z3
   Z3_INCLUDE_DIR: venv_path/include
   Z3_LIBRARY: venv_path/lib/libz3.so
$ make
```

stormpy:
```
$ cd stormpy
$ python3 setup.py develop
```

# Instructions for generating figures:
