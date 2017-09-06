import logging
import numpy
import signal
from functools import partial
from math import sqrt
from multiprocessing import Pool, Pipe, Event
try:
    from tango import DevState, DebugIt
    from tango.server import Device, device_property, attribute, command, \
        DeviceMeta
except ImportError:
    from PyTango import DevState, DebugIt
    from PyTango.server import Device, device_property, attribute, command, \
        DeviceMeta

from pymodelica import compile_fmu
from pyfmi.fmi import load_fmu, FMUException

from ds_tanks.tcp_server import TCPTanksServer
from ds_tanks.tanks_utils import get_model_path, simulate_tanks, \
    run_optimisation, signal_handler, run_linearisation, \
    get_linear_quadratic_regulator, get_squared_error


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
    h1_initial = 20.0
    h2_initial = 20.0
    h3_initial = 20.0
    h1_final = 0.0
    h2_final = 0.0
    h3_final = 0.0
    h1_current = 0.0
    h2_current = 0.0
    h3_current = 0.0
    control_current = 0.0
    init_model = None
    control_value = None
    sim_result = None
    optimal_control = [0.0]
    optimal_h1 = [0.0]
    optimal_h2 = [0.0]
    optimal_h3 = [0.0]
    switch_times = []
    process_pool = Pool()
    my_pipe_end, other_pipe_end = Pipe()
    tcp_process = None
    tcp_kill_event = Event()
    q_lqr = numpy.identity(3)
    r_lqr = 1.0
    k_lqr = [0.0]
    eigenvalues_lqr = [0.0]
    s_lqr = [[0.0], [0.0]]
    verification_error = 0.0
    number_of_elements = 50

    # ----------
    # Properties
    # ----------
    IPOPTTolerance = device_property(dtype=float, default_value=1e-3,
                                     doc="Tolerance of IPOPT solver.")
    ModelFile = device_property(dtype=str, default_value="opt_3_tanks.mop",
                                doc="Name of a file containing model.")
    MaxControl = device_property(dtype=float, default_value=100,
                                 doc="Maximum value of control")
    Tank1Outflow = device_property(dtype=float, default_value=16,
                                   doc="Outflow resistance of the 1st tank.")
    Tank2Outflow = device_property(dtype=float, default_value=16,
                                   doc="Outflow resistance of the 2nd tank.")
    Tank3Outflow = device_property(dtype=float, default_value=18,
                                   doc="Outflow resistance of the 3rd tank.")
    Tank1FlowCoeff = device_property(dtype=float, default_value=0.5,
                                     doc="Flow coefficient of the 1st tank.")
    Tank2FlowCoeff = device_property(dtype=float, default_value=0.5,
                                     doc="Flow coefficient of the 2nd tank.")
    Tank3FlowCoeff = device_property(dtype=float, default_value=0.5,
                                     doc="Flow coefficient of the 3rd tank.")
    SimulationFinalTime = device_property(dtype=float, default_value=50.0,
                                          doc="Final time for a simulation")
    TCPServerEnabled = device_property(dtype=bool, default_value=True,
                                       doc="True if TCP server should be"
                                           "enabled, False if not")
    TCPServerAddress = device_property(dtype=str,
                                       default_value="0.0.0.0:4567",
                                       doc="Address and port (separated by a"
                                           "':') to which the TCP server"
                                           "should be bound.")
    SendControlMode = device_property(dtype=str, default_value='SwitchTimes',
                                      doc="Type of control to be sent to a"
                                          "direct control application. Either"
                                          "of: SwitchTimes or FullTrajectory.")

    # ----------
    # Attributes
    # ----------
    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Level of 1st tank (simulated)")
    def H1Simulated(self):
        return self.h1_sim

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Level of 2nd tank (simulated)")
    def H2Simulated(self):
        return self.h2_sim

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Level of 3rd tank (simulated)")
    def H3Simulated(self):
        return self.h3_sim

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Time of simulation (for plotting)")
    def SimulationTime(self):
        return self.t_sim

    @attribute(dtype=float, label="Optimal time",
               doc="Optimal time obtained from optimisation")
    def OptimalTime(self):
        return self.t_opt

    @attribute(dtype=float, label="H1 Initial", min_value=0.0, max_value=40.0,
               doc="Initial value of level in 1st tank")
    def H1Initial(self):
        return self.h1_initial

    @H1Initial.write
    def H1Initial(self, value):
        self.h1_initial = value

    @attribute(dtype=float, label="H2 Initial", min_value=0.0, max_value=40.0,
               doc="Initial value of level in 2nd tank")
    def H2Initial(self):
        return self.h2_initial

    @H2Initial.write
    def H2Initial(self, value):
        self.h2_initial = value

    @attribute(dtype=float, label="H3 Initial", min_value=0.0, max_value=40.0,
               doc="Initial value of level in 3rd tank")
    def H3Initial(self):
        return self.h3_initial

    @H3Initial.write
    def H3Initial(self, value):
        self.h3_initial = value

    @attribute(dtype=float, label="H1 Final", min_value=0.0, max_value=40.0,
               doc="Final value of level in 1st tank")
    def H1Final(self):
        return self.h1_final

    @H1Final.write
    def H1Final(self, value):
        self.h1_final = value

    @attribute(dtype=float, label="H2 Final", min_value=0.0, max_value=40.0,
               doc="Final value of level in 2nd tank")
    def H2Final(self):
        return self.h2_final

    @H2Final.write
    def H2Final(self, value):
        self.h2_final = value

    @attribute(dtype=float, label="H3 Final", min_value=0.0, max_value=40.0,
               doc="Final value of level in 3rd tank")
    def H3Final(self):
        return self.h3_final

    @H3Final.write
    def H3Final(self, value):
        self.h3_final = value

    @attribute(dtype=float, label="H1 Current",
               doc="Current value of level in 1st tank from direct control")
    def H1Current(self):
        return self.h1_current

    @attribute(dtype=float, label="H2 Current",
               doc="Current value of level in 2nd tank from direct control")
    def H2Current(self):
        return self.h2_current

    @attribute(dtype=float, label="H3 Current",
               doc="Current value of level in 3rd tank from direct control")
    def H3Current(self):
        return self.h3_current

    @attribute(dtype=float, label="Current Control",
               doc="Current value of control from direct control")
    def ControlCurrent(self):
        return self.control_current

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Optimal control obtained from solver")
    def OptimalControl(self):
        return self.optimal_control

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Optimal trajectory of level in 1st tank")
    def OptimalH1(self):
        return self.optimal_h1

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Optimal trajectory of level in 2nd tank")
    def OptimalH2(self):
        return self.optimal_h2

    @attribute(dtype=(float,), max_dim_x=10000,
               doc="Optimal trajectory of level in 3nd tank")
    def OptimalH3(self):
        return self.optimal_h3

    @attribute(dtype=(float,), max_dim_x=100,
               doc="Times of switching between min and max control.")
    def SwitchTimes(self):
        return self.switch_times

    @attribute(dtype=((float,),), max_dim_x=3, max_dim_y=3,
               doc="Matrix of input weights for LQR problem.")
    def Q(self):
        return self.q_lqr

    @Q.write
    def Q(self, value):
        array_value = numpy.asarray(value)
        if not numpy.allclose(array_value, array_value.T):
            raise ValueError("The matrix given is not symmetric!")
        elif numpy.all(numpy.linalg.eigvals(array_value) <= 1e-6):
            raise ValueError("The matrix given doesn't have positive"
                             " eigenvalues!")
        else:
            self.q_lqr = value

    @attribute(dtype=float, min_value=0.0, max_value=1000.0,
               doc="Control weight for LQR problem.")
    def R(self):
        return self.r_lqr

    @R.write
    def R(self, value):
        self.r_lqr = value

    @attribute(dtype=(float,), max_dim_x=20,
               doc="K vector from LQ regulator")
    def K(self):
        return self.k_lqr

    @attribute(dtype=((float,),), max_dim_x=3, max_dim_y=3,
               doc="Solution of Ricatti algebraic equation in LQR problem.")
    def S(self):
        return self.s_lqr

    @attribute(dtype=float, format='e',
               doc="Error of verification. Calculated as a sum of squared"
                   " differences between given and simulated values.")
    def VerificationError(self):
        return self.verification_error

    @attribute(dtype=int, min_value=0, max_value=200,
               doc="Number of elements for Finite Elements Method")
    def FEMElements(self):
        return self.number_of_elements

    @FEMElements.write
    def FEMElements(self, value):
        self.number_of_elements = value

    # TODO: add elements number attribute and add it to optimisation execution

    # ---------------
    # Derived methods
    # ---------------
    def init_device(self):
        super(TanksOptimalControl, self).init_device()
        self.set_state(DevState.OFF)
        self.set_status("Model not loaded.")
        self.model_path = get_model_path(model_file=self.ModelFile)
        self.info_stream("Project path: %s" % self.model_path)
        attributes = ["H1Current", "H2Current", "H3Current",
                      "ControlCurrent"]
        self.set_events_for_attributes(attributes)

        if self.TCPServerEnabled:
            address, port = self.TCPServerAddress.split(':')
            self.debug_stream("Setting up TCP server on %s:%d" % (address,
                                                                  int(port)))
            self.tcp_process = TCPTanksServer(self.other_pipe_end,
                                              address, int(port),
                                              log_level=logging.INFO,
                                              name="TCPServer",
                                              kill_event=self.tcp_kill_event)
            self.tcp_process.start()
        signal.signal(signal.SIGINT, partial(signal_handler,
                                             kill_event=self.tcp_kill_event,
                                             pool=self.process_pool,
                                             process=self.tcp_process,
                                             logger=self.warn_stream))

    def delete_device(self):
        self.tcp_kill_event.set()
        self.process_pool.close()
        self.process_pool.join(1)
        try:
            self.tcp_process.join(2)
        except AttributeError:
            pass
        super(TanksOptimalControl, self).delete_device()

    # --------
    # Commands
    # --------
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
            self.set_state(DevState.STANDBY)
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

    @command
    @DebugIt()
    def RunSimulation(self):
        self.set_state(DevState.ON)
        self.set_status("Launching simulation...")
        checks = self.check_equilibrium(self.control_value or 0.0)
        if False in checks:
            self.control_value = self.get_equilibrium_control()
            self.warn_stream("At least one of levels is not from"
                             " equilibrium, setting control to %f" %
                             self.control_value)
        keyword_args = {'u': self.control_value,
                        't_final': self.SimulationFinalTime,
                        'h10': self.h1_initial,
                        'h20': self.h2_initial,
                        'h30': self.h3_initial,
                        'tank1_outflow': self.Tank1Outflow,
                        'tank2_outflow': self.Tank2Outflow,
                        'tank3_outflow': self.Tank3Outflow,
                        'alpha1': self.Tank1FlowCoeff,
                        'alpha2': self.Tank2FlowCoeff,
                        'alpha3': self.Tank3FlowCoeff}
        res = self.process_pool.apply_async(simulate_tanks,
                                            (self.model_path,), keyword_args,
                                            callback=self.simulation_ended)

    @command
    @DebugIt()
    def RunVerification(self):
        if self.t_opt == -1:
            msg = "Optimisation not yet performed, can't simulate results!"
            self.warn_stream(msg)
            raise Exception(msg)
        else:
            self.set_state(DevState.ON)
            self.set_status("Launching verification...")
            keyword_args = {'u': self.optimal_control,
                            't_final': self.t_opt,
                            'h10': self.h1_initial,
                            'h20': self.h2_initial,
                            'h30': self.h3_initial,
                            'tank1_outflow': self.Tank1Outflow,
                            'tank2_outflow': self.Tank2Outflow,
                            'tank3_outflow': self.Tank3Outflow,
                            'alpha1': self.Tank1FlowCoeff,
                            'alpha2': self.Tank2FlowCoeff,
                            'alpha3': self.Tank3FlowCoeff}
            r = self.process_pool.apply_async(simulate_tanks,
                                              (self.model_path,),
                                              keyword_args,
                                              callback=self.verification_ended)

    @command(dtype_in=bool, doc_in="Use current values of levels as initial "
                                   "values for optimisation?")
    @DebugIt()
    def Optimise(self, use_current_levels):
        self.set_state(DevState.RUNNING)
        self.set_status('Optimisation in progress...')
        if use_current_levels:
            self.info_stream("Using current values of levels as initial "
                             "values.")
            self.h1_initial = self.h1_current
            self.h2_initial = self.h2_current
            self.h3_initial = self.h3_current
        if not self.control_value:
            self.control_value = self.get_equilibrium_control()
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
                                             self.h1_initial,
                                             self.h2_initial,
                                             self.h3_initial,
                                             self.Tank1FlowCoeff,
                                             self.Tank2FlowCoeff,
                                             self.Tank3FlowCoeff,
                                             self.IPOPTTolerance,
                                             0.0,
                                             self.SimulationFinalTime,
                                             self.number_of_elements),
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
            if self.optimal_control[0] not in (0.0, 100.0):
                self.info_stream("Normalising optimal control before sending.")
                self.NormaliseOptimalControl()
        else:
            msg = "The full trajectory mode is not implemented yet."
            self.warn_stream(msg)
            raise NotImplementedError(msg)
        if numpy.array_equal(self.k_lqr, [0.0]):
            self.info_stream("Calculating LQR parameters before sending.")
            self.GetLQR()
        data_for_sending = self.get_data_for_ext_control()
        self.my_pipe_end.send(data_for_sending)

    @command(polling_period=50)
    def GetDataFromDirectControl(self):
        if self.my_pipe_end.poll():
            attributes = {"H1Current": None, "H2Current": None,
                          "H3Current": None, "ControlCurrent": None}
            try:
                received_data = self.my_pipe_end.recv()
                self.h1_current = attributes["H1Current"] = received_data[0]
                self.h2_current = attributes["H2Current"] = received_data[1]
                self.h3_current = attributes["H3Current"] = received_data[2]
                self.control_current = attributes["ControlCurrent"] =\
                    received_data[3]
                self.push_events_for_attributes(attributes)
            except EOFError as e:
                self.debug_stream("No more data from direct control.")
                for attr_name in attributes.keys():
                    attributes[attr_name] = e
                self.push_events_for_attributes(attributes)

    @command
    @DebugIt()
    def GetLQR(self):
        u = self.get_equilibrium_control()
        linear_model = run_linearisation(self.model_path, h10=self.h1_final,
                                         h20=self.h2_final, h30=self.h3_final,
                                         tank1_outflow=self.Tank1Outflow,
                                         tank2_outflow=self.Tank2Outflow,
                                         tank3_outflow=self.Tank3Outflow,
                                         alpha1=self.Tank1FlowCoeff,
                                         alpha2=self.Tank2FlowCoeff,
                                         alpha3=self.Tank3FlowCoeff,
                                         u=u)
        self.debug_stream(repr(linear_model))
        self.debug_stream(linear_model.get_linearisation_point_info())
        k, s, e = get_linear_quadratic_regulator(linear_model, q=self.q_lqr,
                                                 r=self.r_lqr)
        self.k_lqr = k[0]
        self.s_lqr = s
        self.eigenvalues_lqr = e

    @command(dtype_out=str, doc_in="A string representation of eigenvalues of"
                                   " a closed loop system with LQR")
    @DebugIt()
    def GetEigenvaluesFromClosedLoopWithLQR(self):
        return repr(self.eigenvalues_lqr)

    # -------------
    # Other methods
    # -------------
    def set_events_for_attributes(self, attributes):
        for attr_name in attributes:
            if hasattr(self, attr_name):
                self.set_change_event(attr_name, True, False)
                self.set_archive_event(attr_name, True, False)
            else:
                raise AttributeError("No attribute named %s" % attr_name)

    def push_events_for_attributes(self, attributes):
        for attr_name, attr_value in attributes.items():
            if hasattr(self, attr_name):
                self.push_change_event(attr_name, attr_value)
                self.push_archive_event(attr_name, attr_value)
            else:
                raise AttributeError("No attribute named %s" % attr_name)

    def get_equilibrium_control_for_level(self, i):
        outflow = getattr(self, "Tank%dOutflow" % i)
        final_level = getattr(self, "h%d_final" % i)
        control = outflow * sqrt(final_level)
        return control

    @DebugIt(show_ret=True)
    def get_equilibrium_control(self):
        controls = [self.get_equilibrium_control_for_level(1),
                    self.get_equilibrium_control_for_level(2),
                    self.get_equilibrium_control_for_level(3)]
        return sum(controls) / 3.0

    def simulation_ended(self, sim_result):
        self.h1_sim = sim_result['h1']
        self.h2_sim = sim_result['h2']
        self.h3_sim = sim_result['h3']
        self.t_sim = sim_result["time"]
        self.set_state(DevState.STANDBY)
        self.set_status("Simulation complete.")

    def verification_ended(self, sim_result):
        if isinstance(sim_result, FMUException):
            self.set_state(DevState.ALARM)
            self.set_status("Verification failed!")
            self.error_stream(repr(sim_result))
            self.verification_error = 0
        else:
            self.simulation_ended(sim_result)
            wanted_levels = [self.h1_final, self.h2_final, self.h3_final]
            obtained_levels = [self.h1_sim[-1], self.h2_sim[-1],
                               self.h3_sim[-1]]
            self.verification_error = get_squared_error(wanted_levels,
                                                        obtained_levels)

    def set_outflow_values(self):
        self.init_model.set("C1", float(self.Tank1Outflow))
        self.init_model.set("C2", float(self.Tank2Outflow))
        self.init_model.set("C3", float(self.Tank3Outflow))

    def optimisation_ended(self, opt_result):
        opt_success = self.set_optimisation_result(opt_result)
        if opt_success:
            self.set_state(DevState.STANDBY)
            self.set_status("Optimal solution found!")
            self.send_data_if_necessary()
        else:
            self.set_state(DevState.ALARM)
            self.set_status("Optimal solution not found")
            self.t_opt = -1

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
        h1_check = self.h1_final - self.get_equilibrium_level(1, control) < 1e-5
        h2_check = self.h2_final - self.get_equilibrium_level(2, control) < 1e-5
        h3_check = self.h3_final - self.get_equilibrium_level(3, control) < 1e-5
        return [h1_check, h2_check, h3_check]

    @DebugIt(show_args=True, show_ret=True)
    def get_equilibrium_level(self, switch, control):
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

    def get_data_for_ext_control(self):
        final_control_index = 5
        final_switch_time_index = 7
        data = [self.h1_final, self.h2_final, self.h3_final, self.t_opt,
                self.optimal_control[0], 0.0, self.switch_times[0],
                0, self.get_equilibrium_control(), self.k_lqr[0], self.k_lqr[1],
                self.k_lqr[2]]
        try:
            data[final_switch_time_index] = self.switch_times[1]
        except IndexError:
            data[final_switch_time_index] = self.t_opt
        if self.optimal_control[0] not in (0.0, 100.0):
            raise ValueError("Optimal control not normalised (%f)!" %
                             self.optimal_control[0])
        elif self.optimal_control[0] == 0.0:
            data[final_control_index] = self.MaxControl
        return data

    def send_data_if_necessary(self):
        h1_init_w = self.H1Initial.get_write_value()
        print("Write value of H1", h1_init_w)
        if h1_init_w != self.h1_initial:
            self.SendControl()


TANKSOPTIMALCONTROL_NAME = TanksOptimalControl.__name__
run = TanksOptimalControl.run_server

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    run()
