from PyTango import DevState, DebugIt, AttrWriteType
from PyTango.server import Device, device_property, attribute, command, \
    DeviceMeta
from math import sqrt
from pyfmi.fmi import load_fmu

from ds_tanks.tanks_utils import get_model_path, simulate_tanks
from pyjmi import transfer_optimization_problem
from pymodelica import compile_fmu


class TanksOptimalControl(Device):
    """
    TanksOptimalControl is a Tango Device Server for calculating time-optimal
    control of a set of three tanks.
    """
    __metaclass__ = DeviceMeta

    model_path = None
    t_opt = -1.0
    h1_sim = [0.0]
    h2_sim = [0.0]
    h3_sim = [0.0]
    h1_final = 0.0
    h2_final = 0.0
    h3_final = 0.0
    init_model = None
    op = None
    control_value = None
    init_result = None
    optimal_control = [0.0]
    optimal_h1 = [0.0]
    optimal_h2 = [0.0]
    optimal_h3 = [0.0]

    # ----------
    # Properties
    # ----------
    IPOPTTolerance = device_property(dtype=float, default_value=1e-3,
                                     doc="Tolerance of IPOPT solver.")
    ModelFile = device_property(dtype=str, default_value="opt_3_tanks.mop",
                                doc="Name of a file containing model.")
    MaxControl = device_property(dtype=int, default_value=100,
                                 doc="Maximum value of control")
    Tank1Outflow = device_property(dtype=float, default_value=26,
                                   doc="Outflow coefficient of the 1st tank.")
    Tank2Outflow = device_property(dtype=float, default_value=26,
                                   doc="Outflow coefficient of the 2nd tank.")
    Tank3Outflow = device_property(dtype=float, default_value=28,
                                   doc="Outflow coefficient of the 3rd tank.")

    # ----------
    # Attributes
    # ----------
    H1Simulated = attribute(dtype=(float,), max_dim_x=10000,
                            fget="read_h1_simulated",
                            doc="Level of 1st tank (simulated)")
    H2Simulated = attribute(dtype=(float,), max_dim_x=10000,
                            fget="read_h2_simulated",
                            doc="Level of 2nd tank (simulated)")
    H3Simulated = attribute(dtype=(float,), max_dim_x=10000,
                            fget="read_h3_simulated",
                            doc="Level of 3rd tank (simulated)")
    T_opt = attribute(dtype=float, label="Optimal time", fget="read_t_opt",
                      doc="Optimal time from optimisation")
    H1Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H1 Final", fget="read_h1_final",
                        fset="write_h1_final",
                        doc="Final value of level in 1st tank")
    H2Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H2 Final", fget="read_h2_final",
                        fset="write_h2_final",
                        doc="Final value of level in 2nd tank")
    H3Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H3 Final", fget="read_h3_final",
                        fset="write_h3_final",
                        doc="Final value of level in 3rd tank")
    OptimalControl = attribute(dtype=(float,), max_dim_x=10000,
                               fget="read_optimal_control",
                               doc="Optimal control from solver")
    OptimalH1 = attribute(dtype=(float,), max_dim_x=10000,
                          fget="read_optimal_h1",
                          doc="Optimal trajectory of level in 1st tank")
    OptimalH2 = attribute(dtype=(float,), max_dim_x=10000,
                          fget="read_optimal_h2",
                          doc="Optimal trajectory of level in 2nd tank")
    OptimalH3 = attribute(dtype=(float,), max_dim_x=10000,
                          fget="read_optimal_h3",
                          doc="Optimal trajectory of level in 3nd tank")

    # --------
    # Commands
    # --------
    def init_device(self):
        super(TanksOptimalControl, self).init_device()
        self.set_state(DevState.OFF)
        self.set_status("Model not loaded.")
        self.model_path = get_model_path(model_file=self.ModelFile)
        self.info_stream("Project path: %s" % self.model_path)

    @command
    @DebugIt()
    def LoadInitialModel(self):
        """
        Compile the stationary initialization model into an FMU.
        :return: None
        """
        if not self.init_model:
            init_fmu = compile_fmu("TanksPkg.ThreeTanksInit", self.model_path)
            self.init_model = load_fmu(init_fmu)
            self.set_outflow_values()
            self.set_state(DevState.ON)
            self.set_status("Initial model loaded.")
        else:
            msg = "Initial model already loaded."
            self.warn_stream(msg)
            raise TypeError(msg)

    @command
    @DebugIt()
    def ResetModel(self):
        """
        Reset the initial model.
        :return: None
        """
        if self.init_model:
            self.init_model.reset()
            self.set_outflow_values()
        else:
            msg = "Initial model not loaded!"
            self.warn_stream(msg)
            raise TypeError(msg)

    @command(dtype_in=float, doc_in="1st tank final level")
    @DebugIt()
    def FindEquilibriumFromH1(self, h1_final_wanted):
        raise NotImplementedError
    # TODO: Think if this method is necessary (probably not)

    @command(dtype_in=float, doc_in="Control value for model initalisation")
    @DebugIt()
    def GetEquilibriumFromControl(self, control_value):
        self.control_value = control_value
        if not self.init_model:
            self.LoadInitialModel()
        else:
            self.ResetModel()
        self.init_model.set('u', control_value)
        self.init_model.initialize()
        [self.h1_final,
         self.h2_final,
         self.h3_final] = self.init_model.get(['h1', 'h2', 'h3'])

    @command(dtype_out=str, doc_out="Optimisation options in string format")
    @DebugIt()
    def GetOptimisationOptions(self):
        if not self.op:
            msg = "Optimisation problem not yet initalised!"
            self.warn_stream(msg)
            raise TypeError(msg)
        else:
            opt_opts = self.op.optimize_options()
            str_opts = "OPTIMISATION OPTIONS:\n"
            for option, value in opt_opts.items():
                str_opts += '%s: %s\n' % (option, value)
            return str_opts

    @command
    @DebugIt()
    def RunSimulation(self):
        if not self.control_value:
            self.control_value = self.Tank1Outflow * sqrt(self.h1_final)
        checks = self.check_equilibrium(self.control_value)
        if False in checks:
            self.warn_stream("At least one of levels is not from equilibrium")
        self.h1_sim, self.h2_sim, self.h3_sim, self.init_result =\
            simulate_tanks(self.model_path, u=self.control_value)

    @command
    @DebugIt()
    def Optimise(self):
        self.set_state(DevState.RUNNING)
        self.set_status('Optimisation in progress...')
        opt_options = self.prepare_optimisation(self.init_result)
        opt_success = self.run_optimisation(opt_options)
        if opt_success:
            self.set_state(DevState.ON)
            self.set_status("Optimal solution found!")
        else:
            self.set_state(DevState.ALARM)
            self.set_status("Optimal solution not found")

    # -----------------
    # Attribute methods
    # -----------------
    def read_h1_simulated(self):
        return self.h1_sim

    def read_h2_simulated(self):
        return self.h2_sim

    def read_h3_simulated(self):
        return self.h3_sim

    def read_t_opt(self):
        return self.t_opt

    def read_h1_final(self):
        return self.h1_final

    def write_h1_final(self, value):
        self.h1_final = value

    def read_h2_final(self):
        return self.h2_final

    def write_h2_final(self, value):
        self.h2_final = value

    def read_h3_final(self):
        return self.h3_final

    def write_h3_final(self, value):
        self.h3_final = value

    def read_optimal_control(self):
        return self.optimal_control

    def read_optimal_h1(self):
        return self.optimal_h1

    def read_optimal_h2(self):
        return self.optimal_h2

    def read_optimal_h3(self):
        return self.optimal_h3

    # -------------
    # Other methods
    # -------------
    def set_outflow_values(self):
        self.init_model.set("C1", float(self.Tank1Outflow))
        self.init_model.set("C2", float(self.Tank2Outflow))
        self.init_model.set("C3", float(self.Tank3Outflow))

    def prepare_optimisation(self, init_res):
        # 3. Solve the optimal control problem
        # Compile and load optimization problem
        optimisation_model = "TanksPkg.three_tanks_time_optimal"
        self.op = transfer_optimization_problem(optimisation_model,
                                                self.model_path)
        # Set outflow values from properties
        self.op.set("C1", float(self.Tank1Outflow))
        self.op.set("C2", float(self.Tank2Outflow))
        self.op.set("C3", float(self.Tank3Outflow))
        # Set initial values
        self.op.set('h1_final', float(self.h1_final))
        self.op.set('h2_final', float(self.h2_final))
        self.op.set('h3_final', float(self.h3_final))
        self.op.set('u_max', self.MaxControl)

        # Set options
        opt_opts = self.op.optimize_options()
        # opt_opts['n_e'] = 80  # Number of elements
        opt_opts['variable_scaling'] = False
        opt_opts['init_traj'] = init_res
        opt_opts['IPOPT_options']['tol'] = self.IPOPTTolerance
        opt_opts['verbosity'] = 1
        return opt_opts

    def run_optimisation(self, opt_options):
        # Solve the optimal control problem
        res = self.op.optimize(options=opt_options)
        # Extract variable profiles
        self.optimal_h1 = res['h1']
        self.optimal_h2 = res['h2']
        self.optimal_h3 = res['h3']
        self.optimal_control = res['u']
        time_res = res['time']
        self.t_opt = time_res[-1]
        h1_check = self.optimal_h1[-1] - self.h1_final < self.IPOPTTolerance
        h2_check = self.optimal_h2[-1] - self.h2_final < self.IPOPTTolerance
        h3_check = self.optimal_h3[-1] - self.h3_final < self.IPOPTTolerance
        return h1_check and h2_check and h3_check

    @DebugIt(show_args=True, show_ret=True)
    def check_equilibrium(self, control):
        h1_check = self.h1_final - self.get_equilibrium(1, control) < 1e-5
        h2_check = self.h2_final - self.get_equilibrium(2, control) < 1e-5
        h3_check = self.h3_final - self.get_equilibrium(3, control) < 1e-5
        return [h1_check, h2_check, h3_check]

    @DebugIt(show_args=True, show_ret=True)
    def get_equilibrium(self, switch, control):
        outflow = getattr(self, "Tank%dOutflow" % switch)
        eq = control ** 2 / outflow ** 2
        return eq


TANKSOPTIMALCONTROL_NAME = TanksOptimalControl.__name__
run = TanksOptimalControl.run_server

if __name__ == '__main__':
    run()
