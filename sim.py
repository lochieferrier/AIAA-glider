import math
import matplotlib.pyplot as plt


#########################
#WORK TO BE DONE
# FIX PRESSURE CURVE TO REMOVE JUMPS - SIDE EFFECT THESE ARE QUITE GOOD AT PICKING UP TIMING INCONSISTENCIES
# Then Move on To an auto optimizer
################

def printStatus():
 print "aircraftAlt = " + str(aircraftAlt)
 print "aircraftVy = " + str(aircraftVy)
 print "supportAlt = " + str(supportAlt)
 print "supportVy = " + str(supportVy)
 print "t = " + str(t)
 print "missionOpsTime = " + str(missionOpsTime) + "s or " + str(missionOpsTime/60) + " mins"
 print "mission range = " + str((missionOpsTime * aircraftCruiseVelocity)/1000) + "km"
 print "*******"
def plotTrajectories():
 plt.plot(bestAircraftAltList)
 plt.plot(bestSupportAltList)
 plt.ylabel('Altitude (m)')
 plt.title('Simulation Optimized Test')
 plt.xlabel('Time (s)')
 plt.grid(True)
 plt.figtext(.02,.02,'cruisev = ' + str(bestCruiseVelocity) + ', sink = ' + str(bestSinkRate) + ', opstime = ' + str(bestMissionOpsTime) + 's or ' + str(bestMissionOpsTime / 60) + 'mins ' + ', insertalt = ' + str(bestReleaseAltitude) + ', mass - aircraft = ' + str(bestSupportMass)) 

 plt.show()
def updateLists():
 supportVyList.append(supportVy)
 supportVxList.append(supportVx)
 supportAltList.append(supportAlt)
 supportChuteStatusList.append(supportChuteStatus)
 supportMassList.append(supportMass)
 aircraftVyList.append(aircraftVy)
 aircraftVxList.append(aircraftVx)
 aircraftAltList.append(aircraftAlt)
 aircraftInMissionList.append(aircraftInMission)
 pList.append(p)


def calcP(inputAlt):
 if (inputAlt > 7000):
  p = (.699*pow(math.e,-0.00009 * inputAlt))/(.1921 * ((-23.4 - 0.0022 * inputAlt) + 273.1))
 if (inputAlt <= 7000):
  p = (.699*pow(math.e,-0.00009 * inputAlt))/(.1921 * ((-31 - 0.0022 * inputAlt) + 273.1))
 return p


#Figuring out Optimization
MSLEDLList = [[11000,282],[10000,273],[9000,130],[8000,125],[7000,117],[6000,109],[5000,102],[4000,95],[3000,88],[2000,79]]
bestSupportAltList = []
bestAircraftAltList = []
bestMissionOpsTime = 500
bestCruiseVelocity = 0
bestSinkRate = 0
bestReleaseAltitude = 0
bestList = []
count = 0

aircraftMass = 0.2

supportMassBaseMass = 0.5
#17,000 tests
#Altitude iteration

#Support mass iteration
for j in range(0,8):
 supportMass = float(supportMassBaseMass + 0.7 - 0.1*j + aircraftMass)
 #Sink rate iteration
 for k in range (4,30):
  aircraftSinkRate = 1.0/k
 
 #Cruise velocity iteration
 for m in range (0,8):
  mach = float(0.2 + 0.1*m)
  aircraftCruiseVelocity = 244 * mach

  count = count + 1

 ###RUN SIMS#
 supportMass = float(supportMassBaseMass + 0.7 - 0.1*j + aircraftMass)
 initialAlt = 11000
 initialVy = 282
 aircraftSinkRate = 1.0/k

 #Constants
 gravity = 3.7
 safetyDelayTime = 4
 parachuteDelayTime = 3
 aircraftPulloutTime = 5
 supportParachuteDiameter = 2.13
 supportParachuteCd = 2.2
 aircraftMass = 0.2
 commsTowerLandsBeforeAircraft = False

 #Variables
 supportVy = initialVy
 supportVyList = []
 supportVx = 0
 supportVxList = []
 supportAlt = initialAlt
 supportAltList = []
 supportChuteStatus = False
 supportChuteStatusList = []
 supportMassList = []

 aircraftVy = initialVy
 aircraftVyList = []
 aircraftVx = 0
 aircraftVxList = []
 aircraftAlt = initialAlt
 aircraftAltList = []
 aircraftInMission = False
 aircraftInMissionList = []
 pList = []
 t = 0
 missionOpsTime = 0
 p = 0

 #print "Initial conditions"
 #printStatus()
 #Delay Simulation
 
 for i in range(0,safetyDelayTime):
  p = calcP(supportAlt)
  supportVy = supportVy + gravity
  supportAlt = supportAlt - supportVy
  aircraftVy = supportVy
  aircraftAlt = supportAlt
  updateLists()
  t = t + 1
 #print "After initial delay"
 #printStatus()
 #Parachute Deploy Assuming Instantaneous Deceleration

 #Parachute Safety Delay for StabilizatioplotTrajectories()n
 for j in range(safetyDelayTime,parachuteDelayTime + safetyDelayTime):
  p = calcP(supportAlt)
  supportVy = pow((8* supportMass * gravity)/(math.pi * supportParachuteCd * p * pow(supportParachuteDiameter,2)),0.5)
  supportAlt = supportAlt - supportVy
  aircraftAlt = supportAlt
  aircraftVy = supportVy
  updateLists()
  t = t + 1
 #print "After parachute safety delay for stabilization"
 #printStatus()

 supportMass = supportMass - aircraftMass

 #Aircraft Acceleration Phase
 while (aircraftVy < aircraftCruiseVelocity):
  aircraftVy = aircraftVy + aircraftCruiseVelocity
  aircraftAlt = aircraftAlt - aircraftVy
  p = calcP(supportAlt)
  supportVy = pow((8* supportMass * gravity)/(math.pi * supportParachuteCd * p * pow(supportParachuteDiameter,2)),0.5)
  supportAlt = supportAlt - supportVy
  updateLists()
  t = t + 1

 #print "After aircraft acceleration"
 #printStatus()

 currentT = t

 aircraftPulloutRate = (aircraftVy - (aircraftSinkRate*aircraftCruiseVelocity))/aircraftPulloutTime

 ####NEED TO MODEL THIS BETTER#####
 while (t <= currentT + aircraftPulloutTime):
  aircraftVy = aircraftVy - aircraftPulloutRate
  aircraftAlt = aircraftAlt - aircraftVy
  p = calcP(supportAlt)
  supportVy = pow((8* supportMass * gravity)/(math.pi * supportParachuteCd * p * pow(supportParachuteDiameter,2)),0.5)
  supportAlt = supportAlt - supportVy
  updateLists()
  t = t + 1
 

 #Aicraft Glide And Flying Comm Tower Phase
 #glide:
 aircraftVy = aircraftSinkRate * aircraftCruiseVelocity

 #####################################
 #print "After pullout:"
 #printStatus()

 while (aircraftAlt > 0 and supportAlt > 0):
  missionOpsTime = missionOpsTime + 1
  aircraftAlt = aircraftAlt - aircraftVy
  p = calcP(supportAlt)
  supportVy = pow((8* supportMass * gravity)/(math.pi * supportParachuteCd * p * pow(supportParachuteDiameter,2)),0.5)
  supportAlt = supportAlt - supportVy
  if aircraftAlt < 0:
   aircraftAlt = 0
   commsTowerLandsBeforeAircraft = False
  if supportAlt < 0:
   commsTowerLandsBeforeAircraft = True
   supportAlt = 0
  updateLists()
  t = t + 1

 if (missionOpsTime < 1500 and missionOpsTime > bestMissionOpsTime):
  
  if (supportAlt > 50):
   print supportAlt
   print 'found max with ops time: ' + str(missionOpsTime)
   print supportMass
   bestSinkRate = aircraftSinkRate
   bestCruiseVelocity = aircraftCruiseVelocity
   bestMissionOpsTime = missionOpsTime
   bestAircraftAltList = aircraftAltList
   bestSupportAltList = supportAltList
   bestReleaseAltitude = initialAlt
   bestSupportMass = supportMass
 
print 'Cruise velocity: ' + str(bestCruiseVelocity)
print 'Sink rate: ' + str(bestSinkRate)
print 'Mission ops time: ' + str(bestMissionOpsTime) + ' or:' + str(bestMissionOpsTime / 60) + 'mins'
print 'Release alt: ' + str(bestReleaseAltitude)
print 'Support mass without aircraft: ' + str(bestSupportMass)
plotTrajectories()
