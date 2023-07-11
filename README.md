# Light Trace Photonics

See [our website](https://www.ltphotonics.co.uk/) for more details (coming soon!)

A Python Library for interfacing with **Light Trace Photonics** hardware. This module lets you control LTP's automated chip testing rig. It provides 
a main LightRig class which reads in device & measurement information, performs scans, and save data for the user. It also provides two additional classes, one for taking measurements with Thorlabs photodiodes and one for controlling an Osics Mainframe laser. Note, these can be modififed to different laser and phototdiode interfacing if required.

Get in touch with us at
contact@ltphotonics.co.uk.

## Table of contents
- Installation
- Usage
- Credits
- License


## Installation

Requirements:
- A Python 3.6 or greater interpreter.

Dependencies
- `libusb` is required for the backend of pyusb.
- `pyvisa-py` is required to connect to Thorlabs photodiodes.
- `pyusb` is required to connect to Thorlabs photodiodes.
- `pyserial` is required to connect to an Oscis Mainframe laser.
- `qontrol` to interfacing with the required Qontrol M2 driver boards (See [Qontrol's website](https://qontrol.co.uk/) for more details).
- `numpy` is required to perform calculations.

Note, the order in which libusb, pyvisa-py and pyusb are install is important for correct configuration. If you are struggling to connect to the Thorlabs powermeter then type `pyvisa-info` into terminal and check the USB backend is working correctly.

### PyPi Installation

The most stable Qontrol Python API is available through the Python Package Index PyPI. Using a bash terminal or similar, this can be installed as follows:
```bash
pip install ltphotonics
```

### Local Development - Github Installation

The latest development version is available through Github as instructed below.

```bash
git clone https://github.com/ltphotonics/ltphotonics.git
cd ltphotonics/
```

Local installation can be performed using the `setup.py` installation script.

```bash
python setup.py develop
```

## Usage
Contact Light Trace Photonics for documentaiton of usage.

## Credits
List of contributors and authors:
Light Trace Photonics
- Jake Biele
- Dominic Sulway

### Contributions
If you want to contribute to this library contact contact@ltphotonics.co.uk

## License
Copyright (c) 2023 Light Trace Photonics Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


