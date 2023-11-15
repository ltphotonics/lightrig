
from lightrig import LightRig

if __name__ == '__main__':

	# ---------- Technology settings -------------

	# Define powermeter params
	powermeter_serials = ['P0000901', 'PM023647', 'PM022742']
	powermeter_models = ['PM100D','PM100D','PM100USB']
	powermeter_units = ['dBm', 'W', 'mW']
    # NOTE this does not set the units on the PMs, it is intended for readout only

	# Define driver params
	m2_serial_port_name = '/dev/tty.usbserial-XXXXXXX'
	
	# Define laser params
	laser_port_name = '/dev/tty1'
	laser_channel = 'CH4'

	local_optimisation_scan_range_um = 10
	coupling_threshold_dB = -30
	foldername = '...'

	# ---------- Connect to LightRig -------------
	
	device = LightRig(m2_serial_port_name = m2_serial_port_name,
					pd_serials = powermeter_serials,
					pd_models = powermeter_models,
					units = powermeter_units,
					laser_port_name = laser_port_name,
					laser_channel = laser_channel)

	# Read in structure dictionary
	device.read_port_CSV('port_dict_example.csv')

	# Run scan with additional settings
	device.scan(local_optimisation_scan_range_um = local_optimisation_scan_range_um,
				coupling_threshold_dB = coupling_threshold_dB,
				foldername = foldername)
