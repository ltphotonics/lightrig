import lightrig
import viewer
import gdsfactory
import gdspy
from PIL import Image, ImageDraw
import numpy as np

# ---------- Run as normal but with viewer.LayoutViewer, which inherits LightRig class ---------

# Define powermeter params
powermeter_serials = ['P0000901', 'PM023647', 'PM022742']
powermeter_models = ['PM100D','PM100D','PM100USB']
powermeter_units = ['dBm', 'W', 'mW']
# NOTE this does not set the units on the PMs, it is intended for readout only

# Define driver params
m2_serial_port_name = '/dev/tty.usbserial-FT3Z7851'

# Define laser params
laser_port_name = '/dev/tty1'
laser_channel = 'CH4'

# NOTE - You must first convert your gds to a png with pixel size 1x1um, orientated so that the fibre array enters form the top to couple.

# ---------- Connect to LightRig via GUI -------------

gui = viewer.LayoutViewer(gds_png_path='/Users/jakebiele7/Documents/GitHub/ltphotonics/gdspng.png',
                            m2_serial_port_name = m2_serial_port_name)

# gui = viewer.LayoutViewer(gds_png_path = '...',
#                         m2_serial_port_name = m2_serial_port_name,
#                         pd_serials = powermeter_serials,
#                         pd_models = powermeter_models,
#                         units = powermeter_units,
#                         laser_port_name = laser_port_name,
#                         laser_channel = laser_channel)
