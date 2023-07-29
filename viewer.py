"""
Control code for Light Trace Photonics LightRig GUI.

This module lets you run a GUI to control the LightRig.

(c) 2023 Light Trace Photonics Ltd.
"""

from __future__ import division
from __future__ import unicode_literals
from __future__ import print_function
from __future__ import absolute_import
from PIL import Image, ImageTk, ImageDraw

import sys

if sys.version_info.major < 3:
    from builtins import zip
    from builtins import open
    from builtins import int
    from builtins import round
    from builtins import range
    from builtins import super

    from future import standard_library

    standard_library.install_aliases()
else:
    # Python 3 doesn't have basestring, as unicode is type string
    # Python 2 doesn't equate unicode to string, but both are basestring
    # Now isinstance(s, basestring) will be True for any python version
    basestring = str

import os
import re
import colorsys
import warnings
import numpy
import tkinter
import tkinter.messagebox
import tkinter.colorchooser
import lightrig
import pyvisa
import subprocess
import time
import threading

import gdspy
img = None

_stipple = tuple(
    "@" + os.path.join(os.path.dirname(gdspy.__file__), "data", "{:02d}.xbm".format(n))
    for n in range(10)
)
_icon_up = "@" + os.path.join(os.path.dirname(gdspy.__file__), "data", "up.xbm")
_icon_down = "@" + os.path.join(os.path.dirname(gdspy.__file__), "data", "down.xbm")
_icon_outline = "@" + os.path.join(
    os.path.dirname(gdspy.__file__), "data", "outline.xbm"
)
_invisible = 9
_tkinteranchors = [
    tkinter.NW,
    tkinter.N,
    tkinter.NE,
    None,
    tkinter.W,
    tkinter.CENTER,
    tkinter.E,
    None,
    tkinter.SW,
    tkinter.S,
    tkinter.SE,
]

class LayoutViewer(tkinter.Frame, lightrig.LightRig):
    """
    Provide a GUI where the layout can be viewed.

    Parameters
    ----------
    gds_png_path : string
        path to a png of the GDS with one pixel = 1um^2, orientated such that X runs west to east and Y runs north to south.
    frame_rate: integer
        rate at which the gds viewer refreshes (ms), if the gds is large, increase this number to prevent GUI from freezing.
    width : integer
        Horizontal size of the viewer canvas.
    height : integer
        Vertical size of the viewer canvas.
    *args :
        All variables of lightrig.LightRig for connecting to tech

    """

    def __init__(
        self,
        gds_png_path,
        frame_rate=1000,
        width=1200,
        height=2000,
        *args, **kwargs
    ):
        tkinter.Frame.__init__(self, None)
        lightrig.LightRig.__init__(self, *args, **kwargs)

        # Frame rate in ms
        self.frame_rate = frame_rate                      # Refresh rate for the GUI gds viewer
        self.gds_png_path = gds_png_path                  # Relative or absolute path to PNG of gds being tested
        self.last = ''                                    # Internal variable to handle printing to the GUI

        # Red dot & window settings
        self.window_x = 1200                              # Window X size around red dot [um]
        self.window_y = 600                               # Window Y size around red dot [um]
        self.d = 5                                        # Red dot diameter [um]

        #-------------------- Internal parameters - do not change

        self.background="#202020",                        
        self.scan_run_flag = 0
        self.last = ''                                  

        # Define ktinker variables to be extracted from fields in the GUI
        self.csv_dic_path = tkinter.StringVar()
        self._scan_range_um = tkinter.StringVar()
        self._preferred_pm = tkinter.StringVar()
        self._coupling_threshold_dB = tkinter.StringVar()

        # Setup resizable window
        self.grid(sticky="nsew")
        top = self.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        # Setup canvas
        self.canvas = tkinter.Canvas(
            self, width=width, height=height, xscrollincrement=0, yscrollincrement=0
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.configure(bg=self.background)

        # Setup toolbar
        self.frame = tkinter.Frame(self)
        self.frame.columnconfigure(6, weight=1)
        self.frame.grid(row=2, column=0, columnspan=2, padx=2, pady=2, sticky="ew")
        
        # Add bottom canvas for logging, with scroll functionality
        self.l_canvas = tkinter.Canvas(self)
        self.l_canvas.grid(row=3, column=0, rowspan=2, sticky="nsew")
        self.l_scroll = tkinter.Scrollbar(
            self, orient=tkinter.VERTICAL, command=self.l_canvas.yview
        )
        self.l_scroll.grid(row=3, column=2, rowspan=2, sticky="ns")
        self.l_canvas["yscrollcommand"] = self.l_scroll.set

        self.text = tkinter.Text(self.l_canvas, font=("Georgia, 14"), height=5)
        self.text.grid(row=0, column=0, sticky="w")

        # Add some text in the text widget
        self.text.insert(tkinter.END, "Welcome to the LightRig GUI...\n")

        # Setup buttons & input fields
        self.range_label = tkinter.Label(self.frame, text="Local optimisation range [um]")
        self.range_label.grid(row=0, column=0, sticky="w")
        self.range_entry = tkinter.Entry(self.frame,textvariable=self._scan_range_um)
        self.range_entry.grid(row=0, column=1, sticky="w")

        self.preferred_pm_label = tkinter.Label(self.frame, text="Local optimisation channel")
        self.preferred_pm_label.grid(row=0, column=2, sticky="w")
        self.preferred_pm_entry = tkinter.Entry(self.frame,textvariable=self._preferred_pm)
        self.preferred_pm_entry.grid(row=0, column=3, sticky="w")

        self.coupling_threshold_dB_label = tkinter.Label(self.frame, text="Coupling threshold [dBm]")
        self.coupling_threshold_dB_label.grid(row=1, column=0, sticky="w")
        self.coupling_threshold_dB_entry = tkinter.Entry(self.frame,textvariable=self._coupling_threshold_dB)
        self.coupling_threshold_dB_entry.grid(row=1, column=1, sticky="w")

        self.scan_button = tkinter.Button(self.frame, text="Run Scan", command=self._scan_on_thread)
        self.scan_button.grid(row=2, column=0, sticky="w")

        self.local_opt_button = tkinter.Button(self.frame, text="Local optimisation", command=self._local_opt_on_thread)
        self.local_opt_button.grid(row=2, column=1, sticky="w")

        self.path_entry = tkinter.Entry(self.frame, width=42, textvariable=self.csv_dic_path)
        self.path_entry.grid(row=2, column=3, sticky="w")

        self.import_csv = tkinter.Button(self.frame, text="Import device CSV", command=self._import_cvs)
        self.import_csv.grid(row=2, column=2, sticky="w")

        # Load initial GDS image
        global my_img, zoom, draw, my_img_array
        my_img = (Image.open(gds_png_path))
        my_img_array = numpy.asarray(my_img)

        # Add red dot to center
        # Transform LightRig coordinate to gds.png coordinates - PIL placed origin in top left of image
        w, h = my_img.size
        center = [0,0]
        window_x = self.window_x 
        window_y = self.window_y 
        d = self.d

        # Zoom into GDS with red dot in center - starting top left
        zoomed = numpy.asarray(my_img)[int(center[1]):int(center[1]+window_y),
                                    int(center[0]):int(center[0]+window_x)]

        zoom = Image.fromarray(zoomed)
        draw = ImageDraw.Draw(zoom)
        draw.ellipse((window_x/2-d, window_y/2-d, window_x/2+d, window_y/2+d), fill = 'red', outline ='red')
        new_img = ImageTk.PhotoImage(zoom)
        self.canvas_image = self.canvas.create_image(window_x/2, window_y/2, image=new_img, tags = ('GDS coordinates: [{:}, {:}]'.format(self.current_position[0], self.current_position[1])))

        # Add window title and initiate string for printing last message to GUI
        self.master.title("LightRig GUI")
        self.latest = ''

        # Initiate afters comands and start tkinter mainloop
        self.after(self.frame_rate, self._update_canvas)
        self.after(1, self._update_comandline, self.latest)
        self.mainloop()

    # ----------- Internal functions linked to buttons for running LightRig -----------

    def _scan_on_thread(self):
        '''Method to thread scan'''

        thread = threading.Thread(target=self._scan)
        thread.start()

        # Continually update GUI during scan
        self.after(self.frame_rate, self._update_canvas)

        self.after(1, self._update_comandline, self.latest)

    def _local_opt_on_thread(self):
        '''Method to thread local optimisation'''

        thread = threading.Thread(target=self._local_opt)
        thread.start()

        # Continually update GUI during scan
        self.after(self.frame_rate, self._update_canvas)

        self.after(1, self._update_comandline, self.latest)

    def _scan(self):
        '''Method to call LightRig.scan() when button pushed'''

        # Disable buttons
        self.scan_button["state"] = "disabled"
        self.local_opt_button["state"] = "disabled"
        self.import_csv["state"] = "disabled"

        if self._scan_range_um.get() == '' or self._coupling_threshold_dB.get() == '':
            self.latest = 'Please input scan range and coupling threshold before running scan...'
        elif self.scan_run_flag == 1:
            self.latest = 'Please restart to run scan again'
        else:
            self.latest = 'Running scan with local optimisation range {:} um and coupling threshold {:} dBm...'.format(float(self._scan_range_um.get()),int(self._coupling_threshold_dB.get()))
            self.scan_run_flag = 1
            self.scan(local_optimisation_scan_range_um=float(self._scan_range_um.get()),
                                coupling_threshold_dB=float(self._coupling_threshold_dB.get()))

        # Enable buttons
        self.scan_button["state"] = "normal"
        self.local_opt_button["state"] = "normal"
        self.import_csv["state"] = "normal"
        
    def _local_opt(self):
        '''Method to call LightRig._local_optimisation() when button pushed'''

        # Disable buttons
        self.scan_button["state"] = "disabled"
        self.local_opt_button["state"] = "disabled"
        self.import_csv["state"] = "disabled"

        if self._scan_range_um.get() == '' or self._preferred_pm.get() == '':
            self.latest = 'Please input scan range and preferred powermeter before running local optimisation...'
        else:
            self.latest = 'Running local optimisation with scan range {:} um on powermeter channel {:}...'.format(float(self._scan_range_um.get()),int(self._preferred_pm.get()))
            _ = self._local_optimisation(preferred_pm=float(self._preferred_pm.get()),
                                scan_range_um=float(self._scan_range_um.get()))

        # Enable buttons
        self.scan_button["state"] = "normal"
        self.local_opt_button["state"] = "normal"
        self.import_csv["state"] = "normal"

    # Import CSV button
    def _import_cvs(self):
        '''Method to call LightRig.read_port_CSV() when button pushed'''

        # Disable buttons
        self.scan_button["state"] = "disabled"
        self.local_opt_button["state"] = "disabled"
        self.import_csv["state"] = "disabled"

        self.latest = 'Importing device CSV from path: {:}...'.format(str(self.path_entry.get()))
        self.csv_dic_path = str(self.path_entry.get())
        self.read_port_CSV(self.csv_dic_path)

        # Enable buttons
        self.scan_button["state"] = "normal"
        self.local_opt_button["state"] = "normal"
        self.import_csv["state"] = "normal"

    def _update_canvas(self):
        '''Method to update canvas with latest fibre array position'''

        global new_img, test

        # Current position is updated after each move of the stage

        # self.lock.acquire()
        cp_x, cp_y = self.current_position[0], self.current_position[1]
        # self.lock.release()

        if cp_x == 0 and cp_y == 0:
            pass
        else:
            center = numpy.array([cp_x,cp_y])
            window_x = self.window_x 
            window_y = self.window_y 
            d = self.d

            # Transform LightRig coordinate to gds.png coordinates - PIL placed origin in top left of image
            w, h = my_img.size
            center = [-cp_x+w/2, -(-cp_y-h/2)]

            # Deal with edge cases - move dot if we scan into edge of window range
            if int(center[1]-window_y/2) < 0:
                y0 = 0
            else:
                if int(center[1]+window_y/2) >= w:
                    y0 = w-1-window_y
                else:
                    y0 = int(center[1]-window_y/2)

            if int(center[1]+window_y/2) >= w:
                y1 = w-1
            else:
                if int(center[1]-window_y/2) < 0:
                    y1 = int(window_y)
                else:
                    y1 = int(center[1]+window_y/2)

            if int(center[0]-window_x/2) < 0:
                x0 = 0
            else:
                if int(center[0]+window_x/2) >= h:
                    x0 = h-1-window_x
                else:
                    x0 = int(center[0]-window_x/2)

            if int(center[0]+window_x/2) >= h:
                x1 = h - 1
            else:
                if int(center[0]-window_x/2) < 0:
                    x1 = int(window_x)
                else:
                    x1 = int(center[0]+window_x/2)

            # Move dot for edge cases
            if x0 == 0:
                dot_offset_x = int(center[0]-window_x/2)
            elif x1 == h-1:
                dot_offset_x = int(center[0]+window_x/2)
            else:
                dot_offset_x = 0

            if y0 == 0:
                dot_offset_y = int(center[1]-window_y/2)
            elif y1 == w-1:
                dot_offset_y = int(center[1]+window_y/2)
            else:
                dot_offset_y = 0

            zoomed = my_img_array[int(y0):int(y1),
                                        int(x0):int(x1)]
            
            test = Image.fromarray(zoomed)
            draw = ImageDraw.Draw(test)
            draw.ellipse((window_x/2-d+dot_offset_x, window_y/2-d+dot_offset_y, window_x/2+d+dot_offset_x, window_y/2+d+dot_offset_y), fill = 'red', outline ='red')

            new_img = ImageTk.PhotoImage(test)

            # Call after function to start continual update
            self.canvas.itemconfig(self.canvas_image, image=new_img)

        self.after(self.frame_rate, self._update_canvas)


    # Print all log messages continually to the GUI text box
    def _update_comandline(self, message):
        '''Method to update GUI message box with latest command line output'''

        if message == self.last:
            pass
        else:
            self.text.insert(tkinter.END, message + "\n")
            self.last = self.latest
        
        self.after(1, self._update_comandline, self.latest)


        
        

  