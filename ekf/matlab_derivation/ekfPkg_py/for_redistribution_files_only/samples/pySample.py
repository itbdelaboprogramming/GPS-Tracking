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
psi_1dotIn = matlab.double([4.5], size=(1, 1))
VIn = matlab.double([20.5], size=(1, 1))
V_1dotIn = matlab.double([1.1], size=(1, 1))
psi0In = matlab.double([31.4], size=(1, 1))
yOut = my_ekfPkg.ekf(modeIn, dtIn, latIn, lonIn, psi_1dotIn, VIn, V_1dotIn, psi0In)
print(yOut, sep='\n')

my_ekfPkg.terminate()
