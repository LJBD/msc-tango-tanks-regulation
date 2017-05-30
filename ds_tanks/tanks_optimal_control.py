import os

from PyTango import DevState, DebugIt, AttrWriteType
from PyTango.server import Device, device_property, attribute, command, \
    DeviceMeta
from pyfmi.fmi import load_fmu

from ds_tanks.tanks_utils import get_model_path, get_initialisation_values
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

    # ----------
    # Properties
    # ----------
    IPOPTTolerance = device_property(dtype=float, default_value=1e-3,
                                     doc="Tolerance of IPOPT solver.")
    ModelFile = device_property(dtype=str, default_value="opt_3_tanks.mop",
                                doc="Name of a file containing model.")

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
    @DebugIt
    def LoadInitialModel(self):
        """
        Compile the stationary initialization model into an FMU.
        :return: None
        """
        self.debug_stream("In LoadInitialModel")
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
    @DebugIt
    def ResetModel(self):
        """
        Reset the initial model.
        :return: None
        """
        self.debug_stream("In ResetModel")
        if self.init_model:
            self.init_model.reset()
        else:
            msg = "Initial model not loaded!"
            self.warn_stream(msg)
            raise TypeError(msg)

    @command(dtype_in=float, doc_in="1st tank final level")
    @DebugIt
    def FindEquilibriumFromH1(self, h1_final_wanted):
        pass

    @command(dtype_in=float, doc_in="Control value for model initalisation")
    @DebugIt
    def GetEquilibriumFromControl(self, control_value):
        [self.h1_final, self.h2_final, self.h3_final] =\
            get_initialisation_values(self.model_path, control_value)

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
