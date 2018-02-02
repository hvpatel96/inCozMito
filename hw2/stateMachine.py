''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo

currentState = Idle

class State(object):
    action = ""
class Idle(State):
    action =
class Drone(State):
    action =
class Order(State):
    action =
class Inspection(State):
    action =
def transition(next):
    currentState = Idle
    return Idle