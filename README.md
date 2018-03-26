TanksOptimalControl DS
======================

This repository contains TanksOptimalControl DS - a Tango Device Class for
calculating time-optimal control in a system consisting of 3 tanks in a cascade.

Requirements
------------

1. Tango 9.2.x and PyTango 9.2.x
2. JModelica 1.15+

Those can be found within a Docker image in [this repository](https://github.com/ljbd/msc-tango-jmodelica-docker).


What is inside
--------------
1. Optimica model for the system of cascading tanks.
1. Tango Controls Device Server for simulating and optimising said system.
1. Test scripts to automate generating results.

Usage
-----

Before launching the DS, one must set appropriate environment variables given
below:
```console
PYTHONPATH=:/opt/tango/Python/::$PYTHONPATH
JMODELICA_HOME=/opt/tango
IPOPT_HOME=/opt/tango
LD_LIBRARY_PATH=:/opt/tango/lib/:/opt/tango/ThirdParty/Sundials/lib:/opt/tango/ThirdParty/CasADi/lib:$LD_LIBRARY_PATH
SUNDIALS_HOME=/opt/tango/ThirdParty/Sundials
MODELICAPATH=/opt/tango/ThirdParty/MSL
CPPAD_HOME=/opt/tango/ThirdParty/CppAD/
SEPARATE_PROCESS_JVM=/usr/lib/jvm/java-1.8.0-openjdk/
```
Then, launch the DS with:

```console
python tanks_optimal_control.py *DS_INSTANCE_NAME* [-v*LOG_LEVEL*]
```
