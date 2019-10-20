import numpy
import os
numpyPath = numpy.get_include()
print(numpyPath)
command = "sudo ln -s " +  numpyPath + " /usr/include/numpy"
print(command)
os.system(command)
