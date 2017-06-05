from multiprocessing import Pool

from PyTango import DevState, DebugIt, AttrWriteType
from PyTango.server import Device, device_property, attribute, command, \
    DeviceMeta
from math import sqrt

from pyfmi.fmi import load_fmu
from pymodelica import compile_fmu

from ds_tanks.tanks_utils import get_model_path, simulate_tanks, \
    run_optimisation


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
    t_sim = [0.0]
    h1_final = 0.0
    h2_final = 0.0
    h3_final = 0.0
    init_model = None
    control_value = None
    sim_result = None
    optimal_control = [0.0]
    optimal_h1 = [0.0]
    optimal_h2 = [0.0]
    optimal_h3 = [0.0]
    switch_times = []
    process_pool = Pool()

    # ----------
    # Properties
    # ----------
    IPOPTTolerance = device_property(dtype=float, default_value=1e-3,
                                     doc="Tolerance of IPOPT solver.")
    ModelFile = device_property(dtype=str, default_value="opt_3_tanks.mop",
                                doc="Name of a file containing model.")
    MaxControl = device_property(dtype=float, default_value=100,
                                 doc="Maximum value of control")
    Tank1Outflow = device_property(dtype=float, default_value=26,
                                   doc="Outflow coefficient of the 1st tank.")
    Tank2Outflow = device_property(dtype=float, default_value=26,
                                   doc="Outflow coefficient of the 2nd tank.")
    Tank3Outflow = device_property(dtype=float, default_value=28,
                                   doc="Outflow coefficient of the 3rd tank.")
    SimulationFinalTime = device_property(dtype=float, default_value=50.0,
                                          doc="Final time for a simulation")
    ServerAddressPort = device_property(dtype=str,
                                        default_value="localhost:4567",
                                        doc="Address and port (separated by a"
                                            "':') to which the TCP server"
                                            "should be bound.")
    DirectControlAddress = device_property(dtype=str,
                                           doc="Address and port (separated by"
                                               "a ':') of a remote direct"
                                               "control application.",
                                           default_value="localhost:8888")
    SendControlMode = device_property(dtype=str, default_value='SwitchTimes',
                                      doc="Type of control to be sent to a"
                                          "direct control application. Either"
                                          "of: SwitchTimes or FullTrajectory.")

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
    SimulationTime = attribute(dtype=(float,), max_dim_x=10000,
                               fget="read_simulation_time",
                               doc="Time of simulation (for plotting)")
    T_opt = attribute(dtype=float, label="Optimal time", fget="read_t_opt",
                      doc="Optimal time from optimisation")
    H1Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H1 Final", fget="read_h1_final",
                        fset="write_h1_final", min_value=0.0, max_value=40.0,
                        doc="Final value of level in 1st tank")
    H2Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H2 Final", fget="read_h2_final",
                        fset="write_h2_final", min_value=0.0, max_value=40.0,
                        doc="Final value of level in 2nd tank")
    H3Final = attribute(dtype=float, access=AttrWriteType.READ_WRITE,
                        label="H3 Final", fget="read_h3_final",
                        fset="write_h3_final", min_value=0.0, max_value=40.0,
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
    SwitchTimes = attribute(dtype=(float,), max_dim_x=20,
                            fget="read_switch_times",
                            doc="Times of switching between min and max"
                                "control.")

    # --------
    # Commands
    # --------
    def init_device(self):
        super(TanksOptimalControl, self).init_device()
        self.set_state(DevState.OFF)
        self.set_status("Model not loaded.")
        self.model_path = get_model_path(model_file=self.ModelFile)
        self.info_stream("Project path: %s" % self.model_path)
        # TODO: rework state machine model (ON for simulation,
        # RUNNING for optimisation, OFF for anything else?)

    def delete_device(self):
        self.process_pool.join()
        super(TanksOptimalControl, self).delete_device()

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
            raise Exception(msg)

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
            raise Exception(msg)

    @command(dtype_in=float, doc_in="Control value for model initalisation")
    @DebugIt()
    def GetEquilibriumFromControl(self, control_value):
        if not 0 <= control_value <= self.MaxControl:
            msg = "Control not in range <0, %f>" % self.MaxControl
            self.warn_stream(msg)
            raise Exception(msg)
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

    @command(dtype_in=int, doc_in="0 for normal simulation and 1 for"
                                  "simulation with optimal control")
    @DebugIt()
    def RunSimulation(self, switch):
        if switch == 0:
            checks = self.check_equilibrium(self.control_value or 0.0)
            if False in checks:
                ctrl_h1 = self.Tank1Outflow * sqrt(self.h1_final)
                ctrl_h2 = self.Tank2Outflow * sqrt(self.h2_final)
                ctrl_h3 = self.Tank3Outflow * sqrt(self.h3_final)
                self.control_value = (ctrl_h1 + ctrl_h2 + ctrl_h3) / 3.0
                self.warn_stream("At least one of levels is not from"
                                 "equilibrium, setting control to %f" %
                                 self.control_value)
            keyword_args = {'u': self.control_value,
                            't_final': self.SimulationFinalTime,
                            'tank1_outflow': self.Tank1Outflow,
                            'tank2_outflow': self.Tank2Outflow,
                            'tank3_outflow': self.Tank3Outflow}
            res = self.process_pool.apply_async(simulate_tanks,
                                                (self.model_path,),
                                                keyword_args,
                                                callback=self.simulation_ended)
        elif self.t_opt == -1:
            msg = "Optimisation not yet performed, can't simulate results!"
            self.warn_stream(msg)
            raise Exception(msg)
        else:
            keyword_args = {'u': self.optimal_control,
                            't_final': self.t_opt,
                            'tank1_outflow': self.Tank1Outflow,
                            'tank2_outflow': self.Tank2Outflow,
                            'tank3_outflow': self.Tank3Outflow}
            res = self.process_pool.apply_async(simulate_tanks,
                                                (self.model_path,),
                                                keyword_args,
                                                callback=self.simulation_ended)

    @command
    @DebugIt()
    def Optimise(self):
        self.set_state(DevState.RUNNING)
        self.set_status('Optimisation in progress...')
        res = self.process_pool.apply_async(run_optimisation,
                                            (self.model_path,
                                             self.Tank1Outflow,
                                             self.Tank2Outflow,
                                             self.Tank3Outflow,
                                             self.h1_final,
                                             self.h2_final,
                                             self.h3_final,
                                             self.MaxControl,
                                             self.control_value,
                                             self.IPOPTTolerance,
                                             0.0,
                                             self.SimulationFinalTime),
                                            callback=self.optimisation_ended)

    @command
    @DebugIt()
    def NormaliseOptimalControl(self):
        if self.t_opt == -1:
            msg = "Optimisation not yet performed!"
            self.warn_stream(msg)
            raise Exception(msg)
        else:
            for i, ctrl_value in enumerate(self.optimal_control):
                if ctrl_value < 25:
                    self.optimal_control[i] = 0.0
                elif 25 <= ctrl_value < 50:
                    self.warn_stream("Control point %d unusual: %f" %
                                     (i, ctrl_value))
                    self.optimal_control[i] = 0.0
                elif 50 <= ctrl_value < 75:
                    self.warn_stream("Control point %d unusual: %f" %
                                     (i, ctrl_value))
                    self.optimal_control[i] = self.MaxControl
                else:
                    self.optimal_control[i] = self.MaxControl
            self.get_switch_times()

    @command
    @DebugIt()
    def SendControl(self):
        if self.SendControlMode == "SwitchTimes":
            if not self.switch_times:
                self.NormaliseOptimalControl()
    # TODO: add communication with Matlab via TCP handled in a process/thread

    # -----------------
    # Attribute methods
    # -----------------
    def read_h1_simulated(self):
        return self.h1_sim

    def read_h2_simulated(self):
        return self.h2_sim

    def read_h3_simulated(self):
        return self.h3_sim
    
    def read_simulation_time(self):
        return self.t_sim

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

    def read_switch_times(self):
        return self.switch_times

    # -------------
    # Other methods
    # -------------
    def simulation_ended(self, sim_result):
        self.h1_sim = sim_result['h1']
        self.h2_sim = sim_result['h2']
        self.h3_sim = sim_result['h3']
        self.t_sim = sim_result["time"]
        self.set_state(DevState.ON)
        self.set_status("Simulation complete.")

    def set_outflow_values(self):
        self.init_model.set("C1", float(self.Tank1Outflow))
        self.init_model.set("C2", float(self.Tank2Outflow))
        self.init_model.set("C3", float(self.Tank3Outflow))

    def optimisation_ended(self, opt_result):
        opt_success = self.set_optimisation_result(opt_result)
        if opt_success:
            self.set_state(DevState.ON)
            self.set_status("Optimal solution found!")
        else:
            self.set_state(DevState.ALARM)
            self.set_status("Optimal solution not found")

    def set_optimisation_result(self, res):
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

    def get_switch_times(self):
        switch_times = []
        for i in range(len(self.optimal_control) - 1):
            if abs(self.optimal_control[i+1] - self.optimal_control[i]) > 1:
                switch_times.append(i)
        time_step = self.t_opt / len(self.optimal_control)
        self.switch_times = [index * time_step for index in switch_times]


TANKSOPTIMALCONTROL_NAME = TanksOptimalControl.__name__
run = TanksOptimalControl.run_server

if __name__ == '__main__':
    run()
