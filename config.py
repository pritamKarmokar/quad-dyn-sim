# inertia settings
MASS = 4.3259
I_XX = 0.082
I_YY = 0.0845
I_ZZ = 0.1377

# GAINS
K_P_PHI = 100
K_P_THETA = 100
K_P_PSI = 100
K_P_Z = 1
K_D_PHI = 10
K_D_THETA = 10
K_D_PSI = 10
K_D_Z = 10.0


# gravity
ACC_GRAVITY = -9.81

# initial state
PN = 0.0
PE = 0.0
PH = 150.0
U = 0.0
V = 0.0
W = 0.0
PHI = 0.0
THETA = 0.0
PSI = 0.0
P = 0.0
Q = 0.0
R = 0.0

# set acceleration commands
AX = 0
AY = 0.0
AZ = 9.81#-15.6780

# integrator
EULER = 0
RK45 = 1

# time
DELTA_T = 0.01

# other
NUM_INNER_LOOP = 10