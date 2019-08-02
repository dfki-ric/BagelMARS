from mars_interface import *
from euclid import *

def init():
    clearDict()
    # requestSensor("position")
    # requestSensor("rotation")
    # requestSensor("6dof")
    # setConfig("Graphics", "showCoords", 0)
    # setConfig("Scene", "skydome_enabled", 1)
    # #setConfig("Simulator", "calc_ms", 20)
    # setUpdateTime(1)
    # #setConfig("Robo", "behavior", 0)
    # #requestConfig("Robo", "behavior")

    # clearLines("debug")
    # appendLines("debug", 0., 0., 0.2)
    # appendLines("debug", 0., 0., 0.2)
    # configureLines("debug", 5.0, 0, 1, 0)
    setRunning(True)
    print("init python script")
    logMessage("setup python interface")
    return sendDict()

def update(marsData):
    clearDict()
    # close mars
    #logMessage(".")
    quitSim()
    return sendDict()
