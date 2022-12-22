#!/usr/bin/env python
"""
Sample script that uses the ekfPkg module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

from __future__ import print_function
import ekfPkg
import matlab

my_ekfPkg = ekfPkg.initialize()

modeIn = matlab.logical([True], size=(1, 1))
dtIn = matlab.double([0.1], size=(1, 1))
latIn = matlab.double([-6.895369395151447], size=(1, 1))
lonIn = matlab.double([107.6116112416878], size=(1, 1))
odo_VLIn = matlab.double([20.5], size=(1, 1))
odo_VRIn = matlab.double([20.5], size=(1, 1))
yOut = my_ekfPkg.ekf(modeIn, dtIn, latIn, lonIn, odo_VLIn, odo_VRIn)
print(yOut, sep='\n')

my_ekfPkg.terminate()
