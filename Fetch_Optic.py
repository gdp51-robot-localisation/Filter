from opticalflow import Optical_Flow
from time import sleep
from threading import Thread

# Start thread in the main code and call the fetch_optic
# function whenever an updated position is required

optic = Optical_Flow()  # Initiate Thread

def optic_setup():
    optic.start() # Start thread
    sleep(2)

def fetch():
    location=optic.position()
    return location

def reset():
    reset_optic=optic.reset()
    pass
