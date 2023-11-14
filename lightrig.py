"""
Control code for Light Trace Photonics LightRig.

This module lets you control LTP's LightRig, natively in Python, using Thorlabs photodiodes 
& M2 actuators from Qontrol Systems Ltd.

(c) 2023 Light Trace Photonics Ltd.
"""

from __future__ import print_function
import serial, re, time, datetime, os, sys, csv
from collections import deque as fifo
from random import shuffle, randrange
import qontrol
import numpy as np
import pyvisa
import math
from pathlib import Path
import threading

__version__ = "1.0.0"

# Errors realting to the driver, as defined by Qontrol Systems
QONTROL_ERRORS = {
	0:'Unknown error.',
	3:'Power error.',
	4:'Calibration error.',
	5:'Output error.',
	10:'Unrecognised command.',
	11:'Unrecognised input parameter.',
	12:'Unrecognised channel.',
	13:'Operation forbidden.',
	14:'Serial buffer overflow.',
	15:'Serial communication error.',
	16:'Command timed out.',
	17:'SPI error.',
	18:'ADC error.',
	19:'I2C error.',
	30:'Too many errors, some have been suppressed.',
	31:'Firmware trap.',
	90:'Powered up.',
	1:'Out-of-range error.',
	20:'Interlock triggered.'}

# Additional errors relating to automated scanning
LTP_ERRORS = {
	110:'Incorrect device CSV format',
	111:'Not enough powermeters connected to perform measurement',
	112:'Powermeter(s) failed to connect',
	113:'Attempting to run scan with no device dictionary',
	114:'Device out of scan range',
	115:'Local optimisation scan range must be greater than 3.175 um',
	116:'Not enough powermeter connected to complete scan',
	117:'Laser connection failed',
	118:'Laser throwing error during turn on'}
	# ...}

LTP_WARNINGS = {
	401:'Ensure you are couple into device 1 as defined in line 1 of your device dictionary.',
	402:'Ensure M2 channel 0 = X, 1 = Y, Z = 2'}
	# ...}

THOR_ERRORS = {
	201:'Powermeter not found, connection error!',
	202:'Specified unit not recognised'
}

ERRORS = {**QONTROL_ERRORS, **LTP_ERRORS, **LTP_WARNINGS, **THOR_ERRORS}

# Define fatal errors
fatal_errors = [0, 1, 20, 110, 111, 113, 114, 115, 116, 117, 202]

# Log handler as defined by Qontrol Systems
def my_log_handler(err_dict):
	if err_dict['type'] == 'err' and err_dict['id'] in fatal_errors:
		raise RuntimeError('Caught LightRig error {:}: {:} at {:} ms'.format(err_dict['id'], ERRORS[int(err_dict['id'])], 1000*err_dict['proctime']))

# Time stamping for data save
def timestamp():
	return datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S')

# Power unit converter
def dB_to_mW(dB):
	return 10*np.log10(dB)

class LightRig(object):
	"""
	Class which handles tech setup, user interfacing, and data processing.
	
	 pd_models = None            Photodiode device IDs
	 pd_serials = None           Photodiode serial port objects (eg 'COM1', '/dev/tty1')
	 units = None 				 Photodiode units NOTE this does not set the units on the PMs, it is for reading only
	 m2_device_id = None         M2 device IDs
	 m2_serial_port = None       M2 serial port objects
	 m2_serial_port_name = None  Name of M2 port, (eg 'COM1', '/dev/tty1')
	 laser_port_name = None      Name of Oscis mainframe port, (eg 'COM1', '/dev/tty1')
	 laser_channel = None		 Name of Oscis mainframe channel e.g., 'CH4'

	 log = fifo(maxlen = 256)    Log FIFO of communications
	 log_handler = None          Function which catches log dictionaries
	 log_to_stdout = True        Copy new log entries to stdout
	 error_desc_dict             Error code descriptions
	 device_dict = {}            Dictionary of devices to sweep, see port_dict.csv for example layout
	 pms = [None]                List of photodiode devices if already connected
	 laser = None				 Mainframe laser device if already connected

	"""

	def __init__(self, *args, **kwargs):
		"""
		Initialiser.
		"""
		
		# Defaults
		self.pd_models = [None]			    # Thorlabs photodiode models supported 'N7747A; PM100D; PM100USB'
		self.pd_serials = [None]		    # Thorlabs photodiode serials, e.g. 'P2005655'
		self.units = None					# Thorlabs power meter units NOTE this does not set the units and is only for reading purposes

		self.m2_device_id = None		    # Qontrol M2 device ID (i.e. [device type]-[device number])
		self.m2_serial_port = None		    # Qontrol M2 serial port object
		self.m2_serial_port_name = None		# Qontrol M2 name of serial port, eg if WINDOWS then 'COM1' or MAC then '/dev/tty1'

		self.laser_port_name = None         # Laser port number e.g. if WINDOWNS THEN 1 is 'ASRL1::INSTR', if MAC then 1 is '/dev/tty1'
		self.laser_channel = 1	            # Laser channnel e.g. 1 is 'CH1'

		self.log = fifo(maxlen = 4096)		# Log FIFO of sent commands and received errors
		self.log_handler = my_log_handler   # Function which catches log dictionaries
		self.log_to_stdout = True		    # Copy new log entries to stdout

		# Set a time benchmark
		self.init_time = time.time()

		# Initiate device dictionary & list of photodidoe devices
		self.device_dict = {}
		self.pms = [None]			    # List of thorlabs photodiode devices can be passed in
		self.laser = None

		# Define XYZ motor channel defaults
		self.dims = {'X':0, 'Y':1, 'Z':2}

		# Microstep distance for ustep = 0 to 8
		self.dudx = [3.175/2**n for n in [0,1,2,3,4,5,6,7,8]]

		# For GUI
		self.current_position = np.array([0,0])
		self.latest = ''
					
		# Get arguments from init
		# Populate parameters, if provided
		for para in ['pd_models',
					'pd_serials',
					'units',
					'm2_device_id',
					'm2_serial_port_name',
					'laser_port_name',
					'laser_channel',
					'log_to_stdout',
					'device_dict',
					'pms',
					'dims',
					'laser']:
			try:
				self.__setattr__(para, kwargs[para])
			except KeyError:
				continue
		
		# Default powermeter units to dBm
		if self.units is None:
			self.units = ['dBm']*len(self.pd_models)

		# Setup Qontrol tech
		self.qs = qontrol.MXMotor(serial_port_name = self.m2_serial_port_name, log_to_stdout = False, log_handler = self.log_handler, response_timout = 10)
		self.qs.response_timeout = 10
		
		# # # Set motor speed to slow - more accurate translations
		self.qs.set_value(0,'VMAX', 1)
		self.qs.set_value(1,'VMAX', 1)
		self.qs.set_value(2,'VMAX', 1)
		self.qs.set_value(0,'USTEP', 3)
		self.qs.set_value(1,'USTEP', 3)
		self.qs.set_value(2,'USTEP', 3)
		time.sleep(1)

		print ("'{:}' initialised with firmware {:} and {:} channels".format(self.qs.device_id, self.qs.firmware, self.qs.n_chs) )

		# Setup Thorlabs photodiode tech
		if all(element is None for element in self.pms):
			self.pms = []
			for model, serial, unit in zip(self.pd_models, self.pd_serials, self.units):
				try:
					_ = Powermeter(model=model, serial=serial, unit=unit)
					self.pms.append(_)
				except:
					self.log_append(type='err', id='112')

		# Setup laser tech
		if self.laser is None:
			try:
				self.laser = Laser(laser_COM_port = self.laser_port_name, channel = self.laser_channel)
			except:
				self.log_append(type='err', id='117')

		# Perform initial checks
		self.log_append(type='warn', id='402')

		# Create a lock for current position
		self.lock = threading.Lock()

		return

	def read_port_CSV(self, port_dict_filename):
		"""
		Reads in CSV of devices to scan.
		Requires headings:
			id,	port_x_position_um,	port_y_position_um,	number_of_channels, optimisation_channel, wavelength_start_nm, wavelength_stop_nm, steps

        Args:
            arg1: String to csv path.

        Returns:
            A dictionary containing this information

        Raises:
            SpecificException: Description of the exception that can be raised.
        """

		port_dict_filename = Path(port_dict_filename)

		# Sniff CSV delimiter
		with open(port_dict_filename, 'r') as csvfile:

			dialect = csv.Sniffer().sniff(csvfile.read(1024))
			delimiter = dialect.delimiter

		# Open CSV and read values
		reader = csv.DictReader(open(port_dict_filename, 'r'), delimiter=delimiter)
		for row in reader:
			entry = {}
			for key, value in row.items():
				entry[key] = value
			self.device_dict[row['id']] = entry

		# Read data into lists ready for processing
		self.device_ids, self.names, self.port_xs, self.port_ys, self.num_ports, self.opt_port, self.wav_start, self.wav_stop, self.steps = [],[],[],[],[],[],[],[],[]

		for key in self.device_dict:
			try:
				self.device_ids.append(key)
				self.names.append(str(self.device_dict[key]['device_name']))
				self.port_xs.append(float(self.device_dict[key]['port_x_position_um']))
				self.port_ys.append(float(self.device_dict[key]['port_y_position_um']))
				self.num_ports.append(float(self.device_dict[key]['number_of_channels']))
				self.opt_port.append(float(self.device_dict[key]['optimisation_channel']))
				self.wav_start.append(float(self.device_dict[key]['wavelength_start_nm']))
				self.wav_stop.append(float(self.device_dict[key]['wavelength_stop_nm']))
				self.steps.append(float(self.device_dict[key]['steps']))
			except:
				self.log_append(type='err', id='110')

		# Check enough photodiodes are connected to perform measurements
		if np.max(self.num_ports) > len(self.pms):
			self.log_append(type='err', id='116')

		# Calculate device positions relative to origin, defined as the position of device 1
		self.rel_xs = [i-self.port_xs[0] for i in self.port_xs]
		self.rel_ys = [i-self.port_ys[0] for i in self.port_ys]

		self.current_position = np.array([self.port_xs[0],self.port_ys[0]])

		return self.device_dict

	def scan(self, local_optimisation_scan_range_um = 10, coupling_threshold_dB = -30, foldername = Path(os.path.realpath(__file__)).parent/'Results'):

		self.coupling_threshold_dB = coupling_threshold_dB
		self.local_optimisation_scan_range_um = local_optimisation_scan_range_um

		# Switch on laser
		if self.laser is not None:
			try:
				self.laser.switch_on()
			except:
				self.log_append(type = 'err', id = '118')

		# Prepare folder for saving data
		if os.path.exists(foldername):
			foldername += f" {timestamp()}"

		# Check folder exists, if not then make it
		try: 
			os.chdir(foldername)
		except:
			os.mkdir(foldername)

		# Check device dictionary exists
		if not self.device_dict:
			self.log_append(type = 'err', id = '113')

		# Assume coupled into first device
		self.current_position = np.array([self.port_xs[0],self.port_ys[0]])

		self.log_append(type = 'warn', id = '401')

		# Loop through devices to take measurements
		for i in self.device_ids:

			idx = int(i)-1

			# Move to device
			# Log info aobut coupling success
			self.log_append(type='info', id='-1', params='Moving to next component x = {:}, y = {:}'.format(self.rel_xs[idx], self.rel_ys[idx]))
			self._move(XYZ_um = (self.rel_xs[idx], self.rel_ys[idx], 0))

			time.sleep(5)

			# Local optimisation
			self.log_append(type='info', id='-1', params='Begining local optimisation')

			moved = self._local_optimisation(preferred_pm = int(self.opt_port[idx]), scan_range_um = local_optimisation_scan_range_um)

			# Check coupling threshold has been hit
			p_check = self.pms[int(self.opt_port[idx])-1].measure()
			# p = randrange(10)

			if self.pms[int(self.opt_port[idx])-1].unit == 'mW':
				p_check = mW_to_dB(p_check)
			elif self.pms[int(self.opt_port[idx])-1].unit == 'W':
				p_check = mW_to_dB(p_check)*1000
			
			# Log info about coupling success
			self.log_append(type='info', id='-1', params='Max coupling into device {:} = {:.2f} dBm'.format(i, p_check))

			# If not hit, 3x scan range and run optimisation again before throwing a fatal error
			if p_check < coupling_threshold_dB:

				self.log_append(type='info', id='-1', params='Local optimisation failed')

				# Move back
				self.log_append(type='info', id='-1', params='Moving back to original spot and attempting 3x the scan range')
				self._move(XYZ_um = (-moved[0], -moved[1], 0))

				moved = self._local_optimisation(preferred_pm = int(self.opt_port[idx]), scan_range_um = local_optimisation_scan_range_um * 3)

				# Check coupling threshold has been hit
				p_check = self.pms[int(self.opt_port[idx])-1].measure()

				if self.pms[int(self.opt_port[idx])-1].unit == 'mW':
					p_check = mW_to_dB(p_check)
				elif self.pms[int(self.opt_port[idx])-1].unit == 'W':
					p_check = mW_to_dB(p_check)*1000

				self.log_append(type='info', id='-1', params='Max coupling into device {:} from 3x scan area = {:.2f} dBm'.format(i, p_check))

			# If still not hit, assume device has poor coupling at 1550nm and run scan anyway and move on
			if p_check < coupling_threshold_dB:

				self.log_append(type='info', id='-1', params='Local optimisation with 3x scan range failed')
				self.log_append(type='info', id='-1', params='Moving back to original spot')

				self._move(XYZ_um = (-moved[0], -moved[1], 0))
				self.log_append(type='info', id='-1', params='Device {:} coupling failed due to weak signal at wavelength, running wavelength scan anyway and moving to next device...'.format(i))

			# Prep data storge
			labels = ['Wavelength (nm)'] + ['Channel {:} power (dBm)'.format(pw) for pw in range(1,int(self.num_ports[idx]+1))]
			device_data = np.array(labels)

			# Wavelength vector
			wavelengths = np.linspace(self.wav_start[idx],self.wav_stop[idx],int(self.steps[idx]))

			# Scan wavelengths and collect data
			for w in wavelengths:
				
				# set wavelength on laser
				if self.laser is None:
					pass
				else:
					self.laser.set_laser_wavelength(w)
					self.log_append(type='info', id='-1', params='Laser wavelength set to initial value of {} nm'.format(w))
					time.sleep(0.2)

				# Take measurement on each port
				p = [w]

				for port in range(0,int(self.num_ports[idx])):
					
					p.append(self.pms[port].measure())
					# p.append(randrange(100))

				device_data = np.vstack((device_data, p))
				
			self.laser.set_laser_wavelength('1550')
			self.log_append(type='info', id='-1', params='Laser wavelength set to initial value of {} nm'.format('1550'))
			time.sleep(0.5)

			self.device_dict[i].update({'results': device_data})

			# Write data to file incase the scan crashes
			with open(foldername/'id_{:}.csv'.format(i), "w", newline="") as file:
				writer = csv.writer(file)
				writer.writerows(device_data)

			self.rel_xs = [i-self.rel_xs[idx] for i in self.rel_xs]
			self.rel_ys = [i-self.rel_ys[idx] for i in self.rel_ys]

	def _local_optimisation(self, preferred_pm, scan_range_um = 10):
		"""
		Run a local optimisation over X and Y
		For each ustep, create grid and find maximum.
		"""

		# Calculate number of inital grid poitns -> scans grid_N x grid_N, spaced 3.175um apart, centered on the inital position
		grid_N = int(np.ceil(scan_range_um/self.dudx[0]))

		# Hack - if grid_N is odd, then add one to make it even
		if (grid_N % 2 == 1) :
			grid_N += 1

		# Check min scan range
		if scan_range_um < 3.175:
			self.log_append(type='err', id = 115)

		# Step through resolutions, down to a resolution of 3.175um / 2^[usteps[-1]]
		usteps = [0,1]
		grid_N = [grid_N,1]
        
		# X0 = self.qs.x[self.dims['X']]
		# self.qs.x[self.dims['X']] = round(X0)
		# self.qs.wait_until_stopped(t_poll = 0.5)
		# Y0 = self.qs.x[self.dims['Y']]
		# self.qs.x[self.dims['Y']] = round(Y0)
		# self.qs.wait_until_stopped(t_poll = 0.5)  
        
		for u,N in zip(usteps,grid_N):

			# create grid
			stops = [(-N,-N)]
			measurements = []

			for i in range(0,2*N+1):

				i = i%2
				stops.extend([(((-1)**i),0)]*(2*N))
				stops.append((0,1))
			stops.pop()

			for stop in stops:

				# Move X
				X0 = self.qs.x[self.dims['X']]
				self.qs.x[self.dims['X']] = X0 + float(stop[0]*(self.dudx[u]/self.dudx[0]))
				self.qs.wait_until_stopped(t_poll = 0.5)
				time.sleep(np.abs(float(stop[0]*(self.dudx[u]/self.dudx[0]))*0.5))
				X1 = self.qs.x[self.dims['X']]

				# # FOR DEBUGGING WITHOUT M2 PLUGGED IN
				# X0 = self.current_position[0]
				# X1 = self.current_position[0] + stop[0]*(self.dudx[u]/self.dudx[0])
				# if X1 - X0 != float(stop[0]*(self.dudx[u]/self.dudx[0])):
				# 	self.log_append(type='info', id='-1', params='Local optimisation X scan step failed: target move = {:}, X0 = {:}, X1 = {:}'.format(float(stop[0]*(self.dudx[u]/self.dudx[0])), X0, X1))

				Y0 = self.qs.x[self.dims['Y']]
				self.qs.x[self.dims['Y']] = Y0 + float(stop[1]*(self.dudx[u]/self.dudx[0]))
				self.qs.wait_until_stopped(t_poll = 0.5)
				time.sleep(np.abs(float(stop[1]*(self.dudx[u]/self.dudx[0]))*0.5))
				Y1 = self.qs.x[self.dims['Y']]

				# # FOR DEBUGGING WITHOUT M2 PLUGGED IN
				# Y0 = self.current_position[1]
				# Y1 = self.current_position[1] + stop[1]*(self.dudx[u]/self.dudx[0])
				# if Y1 - Y0 != float(stop[1]*(self.dudx[u]/self.dudx[0])):
				# 	self.log_append(type='info', id='-1', params='Local optimisation Y scan step failed: target move = {:}, Y0 = {:}, Y1 = {:}'.format(float(stop[0]*(self.dudx[u]/self.dudx[0])), X0, X1))       
                
				# Update current position with actual distance moved
				self.current_position = np.array([self.current_position[0]+self.dudx[0]*(X0 - X1),self.current_position[1]+self.dudx[0]*(Y1 - Y0)])

				# # Measure
				_ = self.pms[preferred_pm-1].measure()
				measurements.append(_)
				# measurements.append(randrange(100))

			# Move back to optimal spot and switch to next ustep
			p_opt = len(measurements)-np.argmax(measurements)-1

			backwards_stops = [tuple(-x for x in tup) for tup in stops]
			backwards_stops.reverse()

			if u == 0 and p_opt == 0:
				self.log_append(type='info', id='-1', params='Optimal value found at edge of optimisation range - consider increasing scan range')

			move_to = tuple(sum(values) for values in zip(*backwards_stops[0:p_opt]))

			if p_opt == 0:
				pass
			else:

				# Move X
				X0 = self.qs.x[self.dims['X']]
				self.qs.x[self.dims['X']] = X0 + float(move_to[0]*(self.dudx[u]/self.dudx[0]))
				self.qs.wait_until_stopped(t_poll = 0.5)
				time.sleep(np.abs(float(move_to[0]*(self.dudx[u]/self.dudx[0]))*0.5))
				X1 = self.qs.x[self.dims['X']]


				# # FOR DEBUGGING WITHOUT M2 PLUGGED IN
				# X0 = self.current_position[0]
				# X1 = self.current_position[0] + move_to[0]*(self.dudx[u]/self.dudx[0])
				# if X1 - X0 != float(move_to[0]*(self.dudx[u]/self.dudx[0])):
				# 	self.log_append(type='info', id='-1', params='Local optimisation X position step failed: target move = {:}, X0 = {:}, X1 = {:}'.format(float(move_to[0]*(self.dudx[u]/self.dudx[0])), X0, X1))

				Y0 = self.qs.x[self.dims['Y']]
				self.qs.x[self.dims['Y']] = Y0 + float(move_to[1]*(self.dudx[u]/self.dudx[0]))
				self.qs.wait_until_stopped(t_poll = 0.5)
				time.sleep(np.abs(float(move_to[1]*(self.dudx[u]/self.dudx[0]))*0.5))
				Y1 = self.qs.x[self.dims['Y']]

				# # FOR DEBUGGING WITHOUT M2 PLUGGED IN
				# Y0 = self.current_position[1]
				# Y1 = self.current_position[1] + move_to[1]*(self.dudx[u]/self.dudx[0])
				# if Y1 - Y0 != float(move_to[1]*(self.dudx[u]/self.dudx[0])):
				# 	self.log_append(type='info', id='-1', params='Local optimisation Y position step failed: target move = {:}, Y0 = {:}, Y1 = {:}'.format(float(move_to[1]*(self.dudx[u]/self.dudx[0])), X0, X1))          
                
				# Update current position with actual distance moved
				self.current_position = np.array([self.current_position[0]+self.dudx[0]*(X0 - X1),self.current_position[1]+self.dudx[0]*(Y1 - Y0)])

			# Save move incase of failure
			if u == 0:
				if p_opt == 0:
					save_move = ((int(np.ceil(scan_range_um/self.dudx[0])))*self.dudx[0], (int(np.ceil(scan_range_um/self.dudx[0])))*self.dudx[0])
				else:
					save_move = ((int(np.ceil(scan_range_um/self.dudx[0]))+move_to[0])*self.dudx[0], (int(np.ceil(scan_range_um/self.dudx[0]))+move_to[1])*self.dudx[0])

		return save_move
        
		
	def _move(self, XYZ_um = (0,0,0)):
		"""
		For translating distance (x,y,z)um
		"""

		# Axis direction corrections
		XYZ_um = (-XYZ_um[0], XYZ_um[1], XYZ_um[2])
        
		# Move distance
		X0 = self.qs.x[self.dims['X']]
		self.qs.x[self.dims['X']] = X0 + float(XYZ_um[0]/self.dudx[0])
		self.qs.wait_until_stopped(t_poll = 0.5)
		time.sleep(np.abs(float(XYZ_um[0]*(self.dudx[3]/self.dudx[0]))*0.2))

		X1 = self.qs.x[self.dims['X']]

		# if X1 - X0 != float(XYZ_um[0]/self.dudx[0]):
		# 	self.log_append(type='info', id='-1', params='Device stepping X position failed: target move = {:}, X0 = {:}, X1 = {:}'.format(float(XYZ_um[0]/self.dudx[0]), X0, X1))

		Y0 = self.qs.x[self.dims['Y']]
		self.qs.x[self.dims['Y']] = Y0 + float(XYZ_um[1]/self.dudx[0])
		self.qs.wait_until_stopped(t_poll = 0.5)
		time.sleep(np.abs(float(XYZ_um[1]/self.dudx[0]))*0.2)
		Y1 = self.qs.x[self.dims['Y']]

		# if Y1 - Y0 != float(XYZ_um[1]/self.dudx[0]):
		# 	self.log_append(type='info', id='-1', params='Device stepping Y position failed: target move = {:}, Y0 = {:}, Y1 = {:}'.format(float(XYZ_um[1]/self.dudx[0]), Y0, Y1))       
		
		# Update current position with actual distance moved
		self.current_position = np.array([self.current_position[0]+self.dudx[0]*(X0 - X1),self.current_position[1]+self.dudx[0]*(Y1 - Y0)])
				
		return
               
	def log_append (self, type='err', id='', params=''):
		"""
		Log an event; add both a calendar- and process-timestamp.
		"""
		# Append to log fifo
		self.log.append({'timestamp':time.asctime(), 'proctime':round(time.time()-self.init_time,3), 'type':type, 'id':int(id), 'info':str(params)})
		# Send to handler function (if defined)
		if self.log_handler is not None:
			self.log_handler(self.log[-1])
		# Send to stdout (if requested)
		if self.log_to_stdout:
			self.print_log (n = 1)

		# Save for GUI printing
		if id != '-1':
			self.latest = 'Type: {:}, ID: {:}, Info: {:}'.format(type, ERRORS[int(id)], params)
		else:
			self.latest = 'Type: {:}, Info: {:}'.format(type, params)

		print(self.latest)

	def print_log (self, n = None):
		"""
		Print the n last log entries. If n == None, print all log entries.
		"""
		if n is None:
			n = len(self.log)
		
		for i in range(-n,0):

			# -1 marks info logging
			if self.log[i]['id'] == -1:
				print('@ {0: 8.1f} ms, {1} : {2}'.format(1000*self.log[i]['proctime'], self.log[i]['type'], self.log[i]['info']))
			
			# else print error
			else:
				print('@ {0: 8.1f} ms, {1} : {2}'.format(1000*self.log[i]['proctime'], self.log[i]['type'], self.log[i]['id']) + ' ' + ERRORS[int(self.log[i]['id'])], self.log[i]['info'])

class Powermeter(object):
	"""

	Class which handles Thorlabs powermeter setup, user interfacing, and data processing.
	
	 model = None            Photodiode model e.g. 'PM100USB
	 serial = None           Photodiode serial e.g. 'PM000192
	 unit = None         	 Unit settings {'W', 'mW', 'dBm'}
	 wavelength = None       Central wavelength [nm]
	 average = 10  			 Number of measurements to average over [int]
	 read_timeout = 0.5      Readout cut off time [s]

	 log = fifo(maxlen = 256)    Log FIFO of communications
	 log_handler = None          Function which catches log dictionaries
	 log_to_stdout = True        Copy new log entries to stdout

	"""


	def __init__(self, *args, **kwargs):
		"""
		Initialiser.
		"""
		
		# Defaults
		self.model = None		 # Device model, supported 'N7747A; PM100D; PM100USB'
		self.serial = None		 # Name of serial port, eg 'COM1' or '/dev/tty1'
		self.unit = 'dBm'	     # {'W', 'mW', 'dBm'}
		self.wavelength = 1550	 # Wavelength in nm
		self.averages = 10		 # Measurement averages
		self.read_timeout = 0.5  # Read timeout (s)
		
		self.log = fifo(maxlen = 256)   # Log FIFO of communications
		self.log_handler = None         # Function which catches log dictionaries
		self.log_to_stdout = True       # Copy new log entries to stdout
		self.init_time = time.time()

		# Get arguments from init
		# Populate parameters, if provided
		for para in ['model', 'serial', 'unit', 'wavelength', 'averages', 'read_timeout', 'log_to_stdout']:
			try:
				self.__setattr__(para, kwargs[para])
			except KeyError:
				continue

		if self.unit not in ['W', 'dBm', 'mW']:
			self.log_append(type='err', id = 202)
		
		# For
		rm = pyvisa.ResourceManager('@py')
		resources = rm.list_resources()

		pm_serial = None
		instrs = []

		for r in resources:
			if self.serial in r.split('::'):
				pm_serial = r
				instr = rm.open_resource(pm_serial, read_termination = '\n')
				print('Resource is {:}'.format(pm_serial))
	
		# Throw error if connection failed
		if instr == None:
			self.log_append(type='err', id = 201)


	def measure(self, channel = 1):

		if os.name == 'posix':
			r = float(self.instr.query('READ?'))
			result_W = float(self.instr.query('read?;*OPC?', delay=self.read_timeout).split(";")[0])

			if math.isnan(result_W):
				result_W = 0.
			if self.unit == 'W':
				return result_W
			elif self.unit == 'mW':
				return result_W * 1000
			elif self.unit == 'dBm':
				return (10 * math.log( result_W * 1000, 10) ) if result_W > 0 else float('nan')
			
		else:
			if (self.model == 'PM100D' or self.model == 'PM100USB'):
				result_W = float(self.instr.query('MEASure:POWer?'))
			elif (self.model == 'N7747A'):
				if (channel not in [1,2]):
					raise AttributeError ('Channel number for model {0} must be in [1,2]. Specified channel {1} is invalid.'.format(self.model, channel))
				result_W = float(self.instr.query('read{ch}:pow?'.format(ch = channel), delay=self.read_timeout))
			else:
				raise AttributeError('Unknown model "{0}".'.format(self.model))
	
			if math.isnan(result_W):
				result_W = 0.
	
			if self.unit == 'W':
				return result_W
			elif self.unit == 'mW':
				return result_W * 1000
			elif self.unit == 'dBm':
				return (10 * math.log( result_W * 1000, 10) ) if result_W > 0 else float('nan')

	def set_wavelength(self, wl_nm):
		r = self.instr.write('sense:correction:wav {0}'.format(wl_nm))
		sleep(0.005)
		
	def get_wavelength(self):
		r = float(self.instr.query('sense:correction:wav?'))/1e-9
		return r

	def set_averages(self, n_averages):
		self.instr.write('sens:aver {0}'.format(n_averages))
		sleep(0.005)
	
	def get_statistics(self, n_counts = 100, channel = 1):
		data = []
		for _ in range(n_counts):
			power = self.measure(channel = channel)
			if not math.isnan(power):
				data.append(power)
			else:
				return {"mean" : math.nan, "stdev": math.nan,"data": []}
			sleep(0.05)
		if data:
			mean = sum(data)/len(data)
			stats = {
				"min": min(data),
				"max": max(data),
				"mean": mean,
				"stdev": std(array(data), axis=0),
				"data": data
				}
			return stats
		else:
			# only works in dBm for now set to noise floor if NaN
			stats = {
				"min": -120.0,
				"max": -120.0,
				"mean": -120.0,
				"stdev": 0.0,
				"data": []
				}
			return stats

	def log_append (self, type='err', id=''):
		"""
		Log an event; add both a calendar- and process-timestamp.
		"""
		# Append to log fifo
		self.log.append({'timestamp':time.asctime(), 'proctime':round(time.time()-self.init_time,3), 'type':type, 'id':int(id)})
		# Send to handler function (if defined)
		if self.log_handler is not None:
			self.log_handler(self.log[-1])
		# Send to stdout (if requested)
		if self.log_to_stdout:
			self.print_log (n = 1)

	def print_log (self, n = None):
		"""
		Print the n last log entries. If n == None, print all log entries.
		"""
		if n is None:
			n = len(self.log)
		
		for i in range(-n,0):
			print('@ {0: 8.1f} ms, {1} : {2}'.format(1000*self.log[i]['proctime'], self.log[i]['type'], self.log[i]['id']) + ' ' + ERRORS[int(self.log[i]['id'])])


class Laser(object):

	def __init__(self, **kwargs):

		self.laser_COM_port = 1
		self.channel = 1

		# Get arguments from init
		for para in ['laser_COM_port', 'channel']:
			try:
				self.__setattr__(para, kwargs[para])
			except KeyError:
				continue

		rm = pyvisa.ResourceManager('@py')

		if os.name == 'posix':
			self.laser = rm.open_resource(serial = '/dev/tty{:}'.format(self.laser_COM_port))
		else:
			# self.laser = rm.open_resource(serial = "ASRL{}::INSTR".format(self.laser_COM_port))
			self.laser = rm.open_resource(f"ASRL{self.laser_COM_port}::INSTR")
		self.laser.timeout = 20000
		self.laser.read_termination = '\r>'
		self.laser.write_termination = '\r'

		return

	def laser_enable(self):
		response = self.laser.query("CH{:}:ENABLE".format(self.channel)).strip()
		if not response == "CH{:}:OK".format(self.channel):
			raise Exception("Error: {}".format(response))

		return

	def switch_on(self):
		response = self.laser.query("CH{:}:ENABLE".format(self.channel)).strip()
		if not response == "CH{:}:OK".format(self.channel):
			raise Exception("Error: {}".format(response))

		return

	def set_laser_wavelength(self, w = '1550'):

		self.laser_enable()

		# set laser wavelength
		response = self.laser.query("CH{:}:L={}".format(self.channel, w)).strip()
		if not response == "CH{:}:OK".format(self.channel):
			raise Exception("Error: {}".format(response))

		return

class GUI_settings(object):

    def __init__(self):

        self.current_position = np.array([0,0])
        return

    def update_position(self, pos=np.array([0,0])):
        self.current_position = pos
        return