TanksOptimalControl DS
======================

This repository contains TanksOptimalControl DS - a Tango Device Class for
calculating time-optimal control in a system consisting of 3 tanks in a cascade.

Requirements
------------

1. Tango 9.2.x and PyTango 9.2.x
2. JModelica with its requirements:
    1. IPOPT 3.x (the higher the better)
    2. 
TODO: List other requirements

What is inside
--------------
TODO: write short descriptions of the packages

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
