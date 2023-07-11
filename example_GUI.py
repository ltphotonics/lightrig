import lightrig
import viewer
import gdsfactory
import gdspy
from PIL import Image, ImageDraw
import numpy as np

# Function to convert GDS to small PNG for fast videoing
def gds_2_png(gds_path, gds_name, rotate):

    # dr = gds_path+gds_name

    # # Flatten to one top cell
    # @gdsfactory.cell
    # def hash_top(dr):
    #     c = gdsfactory.import_gds(gdspath=dr, cellname=None, flatten=True)
    #     return c
    # t = hash_top(dr)

    # print(t.bbox)
    # t.write_gds(gds_path + 'flattened_' + gds_name)

    # Save flattened top cell as SVG using gdspy - each pixel is 1um x 1um
    gds_object = gdspy.GdsLibrary(infile = gds_path + 'flattened_' + gds_name)
    top = gds_object.extract(list(gds_object.cells.keys())[0])
    top.write_svg('gds.svg', pad = '0%', scaling = 1)

    # Convert large SVG to PNG - still need to figure this bit out!

    return 

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

# Create PNG of GDS for video
gds_path = '/Users/jakebiele7/Desktop/'
gds_name = 'Cornerstone_SOI_ISSIQP_2023_JB_V3.gds'

# gds_2_png(gds_path, gds_name, rotate=180)
# exit()
# ---------- Connect to LightRig via GUI -------------

gui = viewer.LayoutViewer(gds_png_path='/Users/jakebiele7/Documents/GitHub/ltphotonics/gdspng.png',
                            frame_rate=200,
                            m2_serial_port_name = m2_serial_port_name)

# gui = viewer.LayoutViewer(gds_png_path = '...',
#                         frame_rate=1,
#                         m2_serial_port_name = m2_serial_port_name,
#                         pd_serials = powermeter_serials,
#                         pd_models = powermeter_models,
#                         units = powermeter_units,
#                         laser_port_name = laser_port_name,
#                         laser_channel = laser_channel)
