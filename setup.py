#!/usr/bin/env python
import setuptools
from pathlib import Path

this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

__version__ = "1.0.0"

setuptools.setup(
    name="lightrig",
    version=__version__,
    description="Python Library for interfacing with Light Trace Photonics Lightrig.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ltphotonics/lightrig",
    author="Light Trace Photonics Ltd",
    author_email="contact@ltphotonics.co.uk",
    py_modules=["lightrig","viewer"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    install_requires=["libusb", "pyvisa-py", "pyusb", "pyserial", "qontrol", "numpy", "tkinter", "gdspy", "Pillow"],
)
