# *****************************************************************************
# *                                                                           *
# *    CJH 2022 0130 - Code to make the pixy a work with FRC on a pi          *
# *                                                                           *
# *    pass it color and if you need to be NTserver (for debugging), eg.      *
# *    python pixytest.py blue server    OR    python pixytest.py red         *
# *   assumes red is sig 1 and blue is sig 5 - change as needed in the pixy   *
# *     note this only works on linux - using example from                    *
# https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:building_libpixyusb_as_a_python_module_on_linux
# *****************************************************************************

import time
import sys
import numpy as np
import pixy
from ctypes import *
from pixy import *  # Need this because of the BlockArray() c structure
from networktables import NetworkTablesInstance, NetworkTables
import networktables

# get color from command line  - right now accepts red (signature 1) or blue (signature 5), and defaults to red
args = sys.argv[1:]
if len(args) > 0 and args[0] == 'blue':
    sig = 5  # blue
elif len(args) > 0 and args[0] == 'red':  # default color
    sig = 1  # red
else:
    sig = 1  # red

# start NetworkTables client (or server if you passed 'server' on the command line after the color)
team = 2429
team = 2 # on the romi we have to do it this way - the NT server is the desktop since it's running in a sim
ntinst = NetworkTablesInstance.getDefault()
server = True if len(args) > 1 and args[1] == 'server' else False  # get server from command line

if server:
    print("Setting up NetworkTables server")
    ntinst.startServer()
else:
    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)
    ntinst.startDSClient()


# need this to read the pixy data - can I turn this into a list?
class Blocks(Structure):
    _fields_ = [("m_signature", c_uint),
                ("m_x", c_uint),
                ("m_y", c_uint),
                ("m_width", c_uint),
                ("m_height", c_uint),
                ("m_angle", c_uint),
                ("m_index", c_uint),
                ("m_age", c_uint)]


print("Pixy2 Python Example -- Get Blocks (Modified by CJH 2022 0129)")
pixy_init_code = pixy.init()
if pixy_init_code == 0:
    print('Pixy passed init ... should be alive and running')
    pixy_connected = True

else:
    # reconnects do not seem to work - you have to kill python and reload pixy library
    """ print(f'init returned {pixy_init_code} - will attempt to reconnect...')
    pixy_connected = False
    reconnect_count = 0
    while reconnect_count < 100:
        reconnect_count += 1
        time.sleep(5)
        pixy_init_code = pixy.init()
        print(f'Reconnect attempt: {reconnect_count} with result: {pixy_init_code}', end='\r')
        if pixy_init_code == 0:
            pixy_connected = True
            break
    """
    sys.exit('Pixy not found - restart and try again')

pixy_table = ntinst.getTable("Pixy")
pixy_target = pixy_table.getEntry("targets")
pixy_signature = pixy_table.getEntry("sig")
pixy_distance_entry = pixy_table.getEntry("distance")
pixy_rotation_entry = pixy_table.getEntry("rotation")

if pixy_connected:
    pixy.change_prog("color_connected_components")

blocks = BlockArray(10)  # allow up to 10 blocks to be sent
frame = 0

pixy_xres = 315.0
pixy_fov = 60.0

# set up a way to track averages
averages = 5
area_array = np.zeros(averages)
angle_array = np.zeros(averages)
area_array[:] = np.NaN; angle_array[:] = np.NaN

try:
    while pixy_connected:
        count = pixy.ccc_get_blocks(10, blocks)
        # blocks_red = [block for block in blocks if block.m_signature==sig]
        targets = 0
        frame += 1
        if count > 0:
            # found blocks but we can't loop over that c construct easily
            # get the biggest block of the color we want (pixy sorts them for us) - or would we rather have the oldest?
            for index in range(0, count):
                if blocks[index].m_signature == sig:
                    targets = len([t for t in range(count) if blocks[t].m_signature == sig])
                    block = blocks[index]
                    angle = (-1.0 + 2.0*block.m_x/pixy_xres) * pixy_fov/2.0
                    area = block.m_width * block.m_height

                    # implement a let-cycle on the average arrays
                    area_array[:-1] = area_array[1:]; area_array[-1] = area
                    angle_array[:-1] = angle_array[1:]; angle_array[-1] = angle

                    # update the console
                    print(f'[FRAME={frame:6d} BLOCK: SIG={block.m_signature:1d} X={block.m_x:3d} Y={block.m_y:3d} AREA={np.nanmean(area_array):7.1f}'
                          +f' WIDTH={block.m_width:3d} HEIGHT={block.m_height:3d} AGE={block.m_age:3d} ANGLE={angle:.1f}]             ', end='\r')
                    break  # we only want the first one
        else:
            area_array[:] = np.NaN
            angle_array[:] = np.NaN

        if frame % 6 == 0:
            pixy_target.setNumber(targets)
            pixy_signature.setNumber(sig)
            pixy_distance_entry.setNumber(0 if targets == 0 or np.all(np.isnan(area_array)) else np.nanmean(area_array))
            pixy_rotation_entry.setNumber(0 if targets == 0 or np.all(np.isnan(angle_array)) else np.nanmean(angle_array))
            ntinst.flush()

except KeyboardInterrupt:
    print('\nDone')
    # close anything gracefully?
