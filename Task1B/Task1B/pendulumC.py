
#python
import sim
# ###### GLOBAL VARIABLES HERE ######
# base = None
# motor = None
# arm = None
# pendulum = None
# U = None

error_alpha_dot=None
error_alpha = None
error_theta_dot = None
error_theta = None


# You can add variables here 
# as required by your implementation.
###################################

def sysCall_init():
    print("Task1b")
    global base, motor, arm, pendulum,elbow, U 
    global alpha_dot, alpha, theta, theta_dot
    global error_alpha_dot, error_alpha, error_theta_dot, error_theta
    
    #Objects
    base = sim.getObjectHandle('Base_C')
    motor = sim.getObjectHandle('Motor_C')
    arm = sim.getObjectHandle('Arm_C')
    pendulum = sim.getObjectHandle('Pendulum_C')
    elbow = sim.getObjectHandle('Elbow_C')
    
    U = 0.0  # Initialize control input
    
    error_alpha_dot = 0
    error_alpha = 0
    error_theta_dot = 0
    error_theta = 0
    
    

def sysCall_actuation():
    #print("actuation called!")
    global base, motor, arm, pendulum, U 
    global error_alpha_dot, error_alpha, error_theta_dot, error_theta
    
    k = [-1.5092 , -1.0000 , -2.0123 , -9.0810]
    
    x1 = error_alpha_dot # Error in states w.r.t desired setpoint
    x2 = error_alpha
    x3 = error_theta_dot
    x4 = error_theta
    
    U = -k[0]*x1 +k[1]*x2 -k[2]*x3 +k[3]*x4
    # sim.setJointTargetForce(motor, U)
    
def sysCall_sensing():

    global base, motor, arm, pendulum,elbow, U 
    global error_alpha_dot, error_alpha, error_theta_dot, error_theta
    # error = desired - current
    error_alpha_dot = 0 - sim.getJointVelocity(motor)
    error_alpha = 0 - sim.getJointVelocity(motor)
    error_theta_dot = 0 - sim.getJointVelocity(elbow)
    error_theta = 0 - sim.getJointPosition(elbow)
    print(error_alpha)
    

    #################################

def sysCall_cleanup():
    global base, motor, arm, pendulum, U 
    
    # sim.resetDynamicObject(base)
    # sim.resetDynamicObject(motor)
    # sim.resetDynamicObject(arm)
    # sim.resetDynamicObject(pendulum)
