''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo

currentState = Idle

class StateMachine:
    def initialize(self, initialState):
        self.currentState = inititialState
        self.currentState.run()

class State(object):
    def run(self):
        assert 0
    def next(self, input):
        assert 0

class Idle(State):
    def run(self):

class Drone(State):
    def run(self):

class Order(State):
    def run(self):

class Inspection(State):
    def run(self):

def transition(next):
    currentState = Idle
    return Idle

def stateDecider(input):
    return {
        'drone' : Drone
        'order' : Order
        'inspection' : Inspection
        'none' : Idle
    }[input]
def transition(input):
    currentState = stateDecider(input)
    currentState = Idle

