import time, jsbsim
from pymavlink import mavutil

DT = 0.01
fdm = jsbsim.FGFDMExec(None)
fdm.set_root_dir("../")
fdm.load_model("goshawk")
fdm.set_dt(DT)
fdm.run_ic()
