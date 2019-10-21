from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import animation
try: 
  f = open('./build/robotPos.py', 'r')
except IOError:
  print("Error: robotPos.py was not found in the build directory.")
  print("Running the shell-app should produce the robotPos.py file.")
coord = []
record = []
allRecords = []
lines = f.readlines()
dc =  []
for aLine in lines:
  #dc = input("")
  if (aLine == "!\n"):
    # Time for new group of coords
    record.append(coord)
    coord = []
    #print("Current Record:")
    #print(record)
  elif (aLine == "!@#\n"):
    allRecords.append(record)
    # Time to reset coord
    coord = []
    record = []
  else:
    aNum = float(aLine)
    coord.append(aNum)
    #print("Current Coord:")
    #print(coord)

#print(allRecords)

for aRec in allRecords:
  fig = plt.figure(1)
  ax = plt.axes(projection = '3d')
  
  xVec1 = [0, aRec[0][0]]
  yVec1 = [0, aRec[0][1]]
  zVec1 = [0, aRec[0][2]]
   
  plt.plot(xVec1, yVec1, zVec1, 'r')

  xVec2 = [aRec[0][0], aRec[1][0]]
  yVec2 = [aRec[0][1], aRec[1][1]]
  zVec2 = [aRec[0][2], aRec[1][2]]

  plt.plot(xVec2, yVec2, zVec2, 'g')
  
  xVec3 = [aRec[1][0], aRec[2][0]]
  yVec3 = [aRec[1][1], aRec[2][1]]
  zVec3 = [aRec[1][2], aRec[2][2]]

  plt.plot(xVec3, yVec3, zVec3, 'b')

  xVec4 = [aRec[2][0], aRec[3][0]]
  yVec4 = [aRec[2][1], aRec[3][1]]
  zVec4 = [aRec[2][2], aRec[3][2]]

  plt.plot(xVec4, yVec4, zVec4, 'k')

  xVec5 = [aRec[3][0], aRec[4][0]]
  yVec5 = [aRec[3][1], aRec[4][1]]
  zVec5 = [aRec[3][2], aRec[4][2]]

  plt.plot(xVec5, yVec5, zVec5, 'm')

  xVec6 = [aRec[4][0], aRec[5][0]]
  yVec6 = [aRec[4][1], aRec[5][1]]
  zVec6 = [aRec[4][2], aRec[5][2]]

  plt.plot(xVec6, yVec6, zVec6, 'c')

  ax.set_xlim(-4, 4)
  ax.set_ylim(-4, 4)
  ax.set_zlim(-4, 4)

  plt.pause(0.01)
  fig.clear()
  fig.clf()

"""allText = f.read()
allJointPosStr = allText.split("!@#")
config = []
jointGrps = []
for jointGrp in allJointPosStr:
  jointGrps.append(jointGrp.split("\n"))
  
pt = []
for elem in jointGrps:
  elem.remove("")
  print(elem)
  indices = [0]
  indices.append( [i for i,  x in enumerate(elem) if x=="!"])
  print(indices)
  for i in indices:
    for j in indices[1:]:
      print(elem[i : j])
    break
  
  pt.append(elem[0:elem.index("!")])
  t = input(pt)
  pt = []
"""
f.close()
