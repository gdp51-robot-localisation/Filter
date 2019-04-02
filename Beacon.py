from marvelmind_cy_mm import MarvelmindHedge
from time import sleep
import pdb
##################
# Assume this is the main code so thread is made only one time
##################

#   There are two competing factors that cause repeated values:
#       1)  Time taken to scan through data/ finding relevant info = substantial
#           which means that one code is sorting through data while other master code
#           is requesting an updated reading (which has not been calculated yet) i.e.
#           the same value is fetched rather than a new one. This process is repeated
#           until the reading is updated.
#       2)  Sampling update rate of the beacon system @8Hz

hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=4, debug=False) # create MarvelmindHedge thread    

def beacon_setup():
    hedge.start() # start thread
    sleep(3)

def fetch():
    location = hedge.position()
    return location

#beacon_setup()
#f = fetch()
#print(f)
