import numpy
import os
numpyPath = numpy.get_include()
print(numpyPath)
command = "ln -s " +  numpyPath + " /usr/include/numpy"
print(command)
os.sys(command)
