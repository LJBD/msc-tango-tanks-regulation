from PyTango import DevState, DebugIt, AttrWriteType
from PyTango.server import Device, device_property, attribute, command, \
    DeviceMeta
from pyfmi.fmi import load_fmu

from ds_tanks.tanks_utils import get_model_path, get_initialisation_values, \
    simulate_tanks, get_equilibrium
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

    # ----------
    # Properties
    # ----------
    IPOPTTolerance = device_property(dtype=float, default_value=1e-3,
                                     doc="Tolerance of IPOPT solver.")
    ModelFile = device_property(dtype=str, default_value="opt_3_tanks.mop",
                                doc="Name of a file containing model.")
    MaxControl = device_property(dtype=int, default_value=100,
                                 doc="Maximum value of control")

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
                        label="H1 Final", fget="read_h2_final",
                        fset="write_h2_final",
                        doc="Final value of level in 2nd tank")
    H3Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H1 Final", fget="read_h3_final",
                        fset="write_h3_final",
                        doc="Final value of level in 3rd tank")

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
        else:
            msg = "Initial model not loaded!"
            self.warn_stream(msg)
            raise TypeError(msg)

    @command(dtype_in=float, doc_in="1st tank final level")
    @DebugIt()
    def FindEquilibriumFromH1(self, h1_final_wanted):
        pass

    @command(dtype_in=float, doc_in="Control value for model initalisation")
    @DebugIt()
    def GetEquilibriumFromControl(self, control_value):
        [self.h1_final, self.h2_final, self.h3_final] =\
            get_initialisation_values(self.model_path, control_value)

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
        control_value = get_equilibrium(self.h1_final)
        self.h1_sim, self.h2_sim, self.h3_sim, init_res =\
            simulate_tanks(self.model_path, u=control_value)

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

    # -------------
    # Other methods
    # -------------
    def prepare_optimisation(self, init_res):
        # 3. Solve the optimal control problem
        # Compile and load optimization problem
        optimisation_model = "TanksPkg.three_tanks_time_optimal"
        self.op = transfer_optimization_problem(optimisation_model,
                                                self.model_path)
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

    def run_optimisation(self, opt_options):
        # Solve the optimal control problem
        res = self.op.optimize(options=opt_options)

        # Extract variable profiles
        h1_res = res['h1']
        h2_res = res['h2']
        h3_res = res['h3']
        u_res = res['u']
        time_res = res['time']


TANKSOPTIMALCONTROL_NAME = TanksOptimalControl.__name__
run = TanksOptimalControl.run_server

if __name__ == '__main__':
    run()
