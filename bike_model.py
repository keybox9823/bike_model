'''
AUTHOR: Phil Aufdencamp
Desc:
    Longitudinal Vehicle Dynamics Model for a Bicyle and Rider
    Starting with the simplest possible model
        -Point Mass
        -Vacuum
        -Constant Power
    Unless otherwise specified, all units are MKS
Edit History
    3/28/2019
        First Day - main structure is sketched out, constant rider power works
        grade works, gravity works
    3/29/2019
        Relaxing the Point Mass Assumption, small change made a small effect
        Added Air, finish time is 734.6sec

Quality Checks & Results
    Move step size 10x in either direction and make sure the finish time doesn't move
    The above quality check works

Further Complexity Ideas
    Tire-Drag
    Sinusoidal Power-Output
Notes:
    Bead Seat Diameter:
    700C = 622mm, 700mm OD
    650B = 584mm BSD, 650mm OD
'''
import math as m
import copy
import matplotlib
import matplotlib.pyplot as plt


def grade(state):
    #INPUTS: 
    # Vehicle state
    #OUTPUTS:
    # Grade at Distance in %
    return 6 # 6% is hawk Hill

def grade_to_radians(grade):
    #INPUTS:
    # grade in percent
    #OUTPUTS
    # grade in radians
    return m.atan(grade/100)

def pedal_force(state):
    #INPUTS:
    # current state of bike/rider
    #OUTPUTS:
    # Force at the tire in Newtons
    #MATH:
    # P = F*v
    # F = P/v
    power = 250.0 #250 Watts of Power
    if state['velocity'] == 0:
        return 100 # it would throw a divide
        # by zero error otherwise, so this is a handy catch condition
    else:
        return power / state['velocity']

def aero_load(constants, vehicle_params, state):
    #INPUTS:
    # Constants, Vehicle Parameters, Vehicle State
    #OUTPUTS:
    # Aero Load in Newtons
    #MATH:
    # F = 1/2 rho * Cd * A * v^2
    drag = 0.5 * (
        constants['air_density'] * 
        vehicle_params['drag_coefficient'] *
        vehicle_params['frontal_area'] *
        (state['velocity'] ** 2.0)
    )
    return drag
def next_step(constants, vehicle_params, state, step):
    #INPUTS:
    # Vehicle State, increment time
    #OUTPUTS:
    # Vehicle State at t+step
    #MATH:
    # F = ma
    # a = Fnet/m
    #Unpacking a few of the key parameters to make the code a little more readable
    v = state['velocity']
    mass = vehicle_params['point_mass']
    inert = vehicle_params['rotating_inertia']
    r = vehicle_params['wheel_diameter'] / 2.0

    g_force = -1.0 * vehicle_params['point_mass'] * constants['gravity'] * m.sin(
        grade_to_radians(grade(state))) #when grade is + then force is in -X direction
    
    drag = aero_load(constants, vehicle_params, state)

    net_force = pedal_force(state) + g_force - drag
    time_next = state['time'] + step
    distance_next = (
        state['distance'] 
        + (state['velocity'] * step))
    velocity_next = m.sqrt(
        v ** 2 + (2 * net_force * v * step / 
        (mass + inert/(r ** 2)))
    )
    next_state = {
        'time' : time_next,
        'distance' : distance_next,
        'velocity' : velocity_next
    }
    return next_state

def solver(constants, course_params, vehicle_params, initial_state, step):
    #INPUTS:
    # Course Definition
    # Vehicle Definition
    # Initial State
    # Time Step
    #OUTPUTS
    # Time Series of all state variables
    state = copy.deepcopy(initial_state)
    time_series = {}
    for key in initial_state:
        time_series[key] = [0.0]
    
    while state['distance'] < course_params['course_distance']:
        state = next_step(constants, vehicle_params, state, step)
        for key in state:
            time_series[key].append(state[key])
    return time_series

if __name__ == "__main__":
    step = 0.1 #seconds
    course_params = {
        "course_distance" : 2656, #m
    }
    wheel_bsd = .584 # m
    wheel_mass = 3.1 # kg
    inertia = 0.25 * wheel_mass * wheel_bsd ** 2
    vehicle_params = {
        'point_mass' : 112.0, # Phil + loaded rove in kg minus Wheels + Tires
        'rotating_inertia' : inertia, #wheels & tires
        'wheel_diameter' : .65, # m, 650B
        'drag_coefficient' : 1.0, # Unitless
        'frontal_area' : .38 # m^2
    }
    constants = {
        'gravity' : 9.81, #m/s^2
        'air_density' : 1.225 #kg / m^3
    }
    initial_state = {
        "time" : 0.0, # sec
        "distance" : 0.0, # m
        "velocity" : 0.1 # m/s
    }

    time_series = solver(constants, course_params,vehicle_params, initial_state, step)

    fig, ax = plt.subplots(2,1)
    
    ax[0].plot(time_series['time'], time_series['distance'])
    ax[1].plot(time_series['time'], time_series['velocity'])

    ax[0].set(xlabel='time [s]', ylabel='distance [m]',
        title='Distance vs Time')
    ax[1].set(xlabel='time [s]', ylabel='velocity [m/s]',
        title='Velocity vs Time')

    ax[0].grid()
    ax[1].grid()

    fig.savefig("time_series.png")

    print('Finish time is', time_series['time'][-1],'seconds')
    