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
    3/28/2020
        First Day - main structure is sketched out, constant rider power works
        grade works, gravity works
    3/29/2020
        Relaxing the Point Mass Assumption, small change made a small effect
        Added Air, finish time is 734.6sec
    4/5/2020
        Refactoring the code into something object oriented. 
        Verified that it produced equivalent results

Quality Checks & Results
    Move step size 10x in either direction and make sure the 
    finish time doesn't move
    The above quality check works

Further Complexity Ideas
    Tire-Drag
    Sinusoidal Power-Output
    Headwind + Tailwind
    Powertrain Efficiency

Implementation / Software Improvements Ideas
    Transition to object oriented structure
    Make Course.grade a little more elegant
Notes:
    Bead Seat Diameter:
    700C = 622mm BSD, 700mm OD
    650B = 584mm BSD, 650mm OD
'''
import math as m
import copy
import matplotlib
import matplotlib.pyplot as plt

class BikeModel:
    ''' A Bike Model Object that can be used to 
    create a time series result of a simulation
    '''

    def __init__(self, time_step, course, vehicle, state):
        self.course = course
        self.time_step = time_step
        self.state = state
        self.vehicle = vehicle
        self.time_series = state.get_state_variables(course, vehicle)
        for key in self.time_series:
            self.time_series[key] = [0.0]

    def run(self):
        next_row = {}

        while self.state.distance < self.course.length:
            next_row = self.state.advance(
                self.time_step, self.course, self.vehicle)
            for key in next_row:
                self.time_series[key].append(next_row[key])
        return self.time_series

class State:
    def __init__(self, start_time, start_velocity, start_distance):
        self.time = start_time
        self.distance = start_distance
        self.velocity = start_velocity
    
    def advance(self, tstep, course, vehicle):
        # Advances the state variables, and returns all things we are 
        # tracking in a dictionary
        self.time += tstep
        v = self.velocity
        mass = vehicle.mass
        inert = vehicle.rotating_inertia
        r = vehicle.wheel_diameter / 2.0

        net_force = (
            vehicle.gravity_force(course.grade(self.distance))
            + vehicle.drag_force(self.velocity)
            + vehicle.pedal_force(self) #so that's cool, an object 
            # can pass itself as an argument to another function
        )
        self.distance += self.velocity * tstep
        self.velocity = m.sqrt(
        v ** 2 + (2 * net_force * v * tstep / 
        (mass + inert/(r ** 2)))
    )
        return {
            'time': self.time, 
            'distance': self.distance, 
            'velocity': self.velocity, 
            'grade': course.grade(self.distance),
            'gravity_force': vehicle.gravity_force(course.grade(self.distance)),
            'drag_force': vehicle.drag_force(self.velocity),
            'pedal_force': vehicle.pedal_force(self)
        }
    
    def get_state_variables(self, course, vehicle):
        # Returns a list of the current state of all the variables we are tracking
        return {
            'time': self.time, 
            'distance': self.distance, 
            'velocity': self.velocity, 
            'grade': course.grade(self.distance),
            'gravity_force': vehicle.gravity_force(course.grade(self.distance)),
            'drag_force': vehicle.drag_force(self.velocity),
            'pedal_force': vehicle.pedal_force(self)
        }

class Course:
    ''' Course Object that will be fed into a BikeModel object
    '''
    def __init__(self, length, grade_vs_distance):
        # length = Length in meters of the total course
        # grade_vs_distance = Dictionary with segments defined as each 
        # end point of a grade segment
        self.length = length
        self.grade_vs_distance = grade_vs_distance
    
    def grade(self, distance):
        # using the grade_vs_distance, returns the grade in percentage 
        # at any arbitrary distance given
        # This can be implemented more cleanly, not sure how yet
        i = 0
        for x in self.grade_vs_distance['distance_points']:
            if distance <= x:
                return self.grade_vs_distance['grade_points'][i]
            i += 1
        return -200 #designed to be a broken value

class Vehicle:
    '''Defines a bike+rider Vehicle Object, holds all the variables & info
    needed to calculate the various forces & what not
    '''
    # It doesn't really make sense symbollically to 
    # put the constants here, but it makes the implementation more logical

    AIR_DENSITY = 1.225 #kg / m3. 
    GRAVITY = 9.81 #m/s

    def __init__(self, mass, wheel_mass, frontal_area, drag_coefficient, peak_power):
        self.wheel_bsd = .584 #Bead Seat Diameter in m, 700c Wheels
        self.wheel_diameter = .650 #m, 650B
        self.mass = mass
        self.wheel_mass = wheel_mass
        self.rotating_inertia = 0.25 * wheel_mass * self.wheel_bsd ** 2
        self.frontal_area = frontal_area
        self.drag_coefficient = drag_coefficient
        self.peak_power = peak_power
    
    def pedal_force(self, state):
        #INPUTS:
        # current state of bike/rider
        #OUTPUTS:
        # Force at the tire in Newtons
        #MATH:
        # P = F*v, F = P/v
        if state.velocity == 0:
            return 100
        else:
            return self.peak_power/state.velocity
    
    def drag_force(self, air_speed):
        # Using air speed instead of velocity to allow us to build 
        # in head/tail winds later
        return -0.5 * (
            self.AIR_DENSITY
            * self.drag_coefficient
            * self.frontal_area
            * air_speed ** 2
        )
    
    def gravity_force(self, grade):
        return -1.0 * (
            self.mass
            * self.GRAVITY
            * m.sin(m.atan(grade/100)) # need to convert grade in % to radians
        )

if __name__ == "__main__":
    time_step = 0.1 # seconds
    hawk_hill_length = 2656 # meters
    hawk_hill_profile = { # this says from 0 - 2800m, grade is 6%
        'distance_points': [0,2800], # want to make sure the grade profile is 
        # defined on a larger domain than the course itself
        'grade_points': [0,6]
    }
    # Create the various simulation objects
    hawk_hill = Course(hawk_hill_length, hawk_hill_profile)
    phil_rove = Vehicle(115.0, 3.1, 0.38, 1.0, 250.0)
    start_state = State(0.0,0.1,0.0)
    hawk_phil_rove = BikeModel(time_step, hawk_hill, phil_rove, start_state)
    time_data = hawk_phil_rove.run()

    fig, ax = plt.subplots(2,1)
    
    ax[0].plot(time_data['time'], time_data['distance'])
    
    ax[1].plot(time_data['time'], time_data['velocity'])

    ax[0].set(xlabel='time [s]', ylabel='distance [m]',
        title='Distance vs Time')
    ax[1].set(xlabel='time [s]', ylabel='velocity [m/s]',
        title='Velocity vs Time')

    ax[0].grid()
    ax[1].grid()

    fig.savefig("time_data.png")

    print('Finish time is', time_data['time'][-1],'seconds')

    print('Finish velocity is ', time_data['velocity'][-1], 'meters/second')
    