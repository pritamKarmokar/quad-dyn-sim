from easydict import EasyDict as edict

__all__ = ['cfg']

__C = edict()
cfg = __C

# inertia settings
__C.INERTIA = edict()
__C.INERTIA.MASS = 4.3259
__C.INERTIA.I_XX = 0.082
__C.INERTIA.I_YY = 0.0845
__C.INERTIA.I_ZZ = 0.1377

# GAINS
__C.GAINS = edict()
__C.GAINS.K_P_PHI = 100
__C.GAINS.K_P_THETA = 100
__C.GAINS.K_P_PSI = 100
__C.GAINS.K_P_Z = 1
__C.GAINS.K_D_PHI = 10
__C.GAINS.K_D_THETA = 10
__C.GAINS.K_D_PSI = 10
__C.GAINS.K_D_Z = 10.0

# gravity
__C.ENV = edict()
__C.ENV.ACC_GRAVITY = 9.81

# initial state
__C.INIT_STATE = edict()
__C.INIT_STATE.PN = 0.0
__C.INIT_STATE.PE = 0.0
__C.INIT_STATE.PH = 150.0
__C.INIT_STATE.U = 0.0
__C.INIT_STATE.V = 0.0
__C.INIT_STATE.W = 0.0
__C.INIT_STATE.PHI = 0.0
__C.INIT_STATE.THETA = 0.0
__C.INIT_STATE.PSI = 0.0
__C.INIT_STATE.P = 0.0
__C.INIT_STATE.Q = 0.0
__C.INIT_STATE.R = 0.0

# set acceleration commands
__C.ACC_COMMAND = edict()
__C.ACC_COMMAND.AX = 1.0
__C.ACC_COMMAND.AY = 0.0
__C.ACC_COMMAND.AZ = 0.0 #-15.6780

# integrators
__C.INTEGRATOR = edict()
__C.INTEGRATOR.EULER = 0
__C.INTEGRATOR.RK45 = 1

# time
__C.TIME = edict()
__C.TIME.FINAL_TIME = 0.5
__C.TIME.DELTA_T = 0.01

# other
__C.SIM = edict()
__C.SIM.NUM_INNER_LOOP = 10