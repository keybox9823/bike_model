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
        return power/state['velocity']

def next_step(constants, state, vehicle_params, step):
    #INPUTS:
    # Vehicle State, increment time
    #OUTPUTS:
    # Vehicle State at t+step
    #MATH:
    # F = ma
    # a = Fnet/m
    g_force = vehicle_params['mass'] * constants['gravity'] * m.sin(
        grade_to_radians(grade(state)))
    net_force = pedal_force(state) - g_force
    acceleration = net_force / vehicle_params['mass']
    time_next = state['time'] + step
    distance_next = (
        state['distance'] 
        + (state['velocity'] * step))
    velocity_next = state['velocity'] + acceleration*step
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
        state = next_step(constants, state, vehicle_params, step)
        for key in state:
            time_series[key].append(state[key])
    return time_series

if __name__ == "__main__":
    step = 0.1 #seconds
    course_params = {
        "course_distance" : 2656, #m
    }
    vehicle_params = {
        'mass' : 115 # Phil + loaded rove in kg
    }
    constants = {
        'gravity' : 9.81 #m/s^2
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
    