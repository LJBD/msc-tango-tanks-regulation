import os

print os.environ
from pymodelica import compile_jmu
from pyjmi import JMUModel
import matplotlib.pyplot as plt

# Compiling a JMU- object for optimization based on the double integrator
os.environ["MODELICAPATH"] += "../optimica_model"
jmu_name= compile_jmu("opt_3_tanks", ["opt_3_tanks.mop"])
# Loading the JMU-object
model_opt = JMUModel(jmu_name)
# Calling the optimisation function with
# default settings
res = model_opt.optimize()

# Plotting the results
h1 = res['h1']
h2 = res['h2']
h3 = res['h3']
# u is the optimal trajectory
u =res['u']
time = res['time']
plt.plot(time, h1, ':k', time, h2, '--k', time, u, '-k')
plt.grid(True)
# Increasing the plot window to show results
plt.ylim([-1.5,2])
plt.legend(('h1', 'h2', 'u'))
plt.title('Simple example')
plt.show()
