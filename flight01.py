import spidev
import time
import sys
import navio.mpu9250
import navio.util
import navio.ublox
import math 
import numpy as np
import inspect

navio.util.check_apm()
imu = navio.mpu9250.MPU9250()
imu.initialize()
time.sleep(1)

class BerryBoi():  
  lat = 0.0
  lon = 0.0
  turn = " " #What direction the payload needs to turn
  speed = 0.0
  error = 0.0 #Angle of correction needed for proper trajectory
  bearing = 0.0
  GPStime = 0.0
  target = [-122, 36.6] #Target position
  angles = [0.0,0.0,0.0,0.0] #Angular change between coordinates
  distances = [0.0,0.0,0.0,0.0] #Distance between coordinates
  matr = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]])
  
  def getData(self, wait): #Collects four (could be any number) GPS coordinates to be used for later calculations, with an adjustable wait time between each collection to allow for enough displacement
    ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
    ubl.configure_poll_port()
    ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
    #ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

    ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
    ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
    ubl.configure_poll_port(navio.ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=1000)

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
    """
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)
    """
    timeyBoi = time.time()
    countyBoi = 0
    x = False
    
    try:
      while (x == False):
        report = ubl.recieve_message()

        if report.name()== "NAV_POSLLH":
          time.sleep(0.1)
          print "Searching for GPS data \n"
          
          self.lon = int(getattr(report,'longitude', 0.0))
          self.lat = int(getattr(report,'latitude', 0.0))
          
          if (self.lat != 0.0 and self.lon != 0.0 and time.time() - timeyBoi >= wait):
            self.matr[countyBoi, 0] = math.radians(self.lon)
            self.matr[countyBoi, 1] = math.radians(self.lat)
            countyBoi += 1
            timeyBoi = time.time()
            print "Reference longitude {}: ".format(countyBoi), self.lon
            print "Reference latitude {}: ".format(countyBoi), self.lat
            print "Waiting 15 seconds to collect more data...\n"

          if (countyBoi == 3 and self.lat != 0.0 and self.lon != 0.0):
            self.matr[3, 0] = math.radians(self.lon)
            self.matr[3, 1] = math.radians(self.lat)
            print "Current longitude: ", self.lon
            print "Current latitude: ", self.lat
            x = True

    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
      print "Done.\nExiting."

  def getDistance(self): #Uses haversine formula to get arc length between GPS coordinates collected in 'matr' (earth is assumed to be spherical, so there is an error factor of %0.03, which shouldn't matter for our purposes)
    r_earth = 6371
    for i in range(4):
      if (i < 3):
        a = math.sin((self.matr[i+1,1] - self.matr[i,1])/2)**2 + math.cos(self.matr[i+1,1])*math.cos(self.matr[i,1])*math.sin((self.matr[i+1,0] - self.matr[i,0])/2)
      else:
        a = math.sin((self.target[1] - self.matr[i,1])/2)**2 + math.cos(self.target[1])*math.cos(self.matr[i,1])*math.sin((self.target[0] - self.matr[i,0])/2)
      c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
      self.distances[i] = c * r_earth
  
  def getRelBearing(self): #Calculates 'relative bearing' of course path using GPS coordinates collected in 'matr' (assumes the payload is facing forward in relation to velocity vector and course path, we will check this with accelerometer)
    for i in range(4):
      if (i < 3):
        x = math.cos(self.matr[i+1,1])*math.sin(self.matr[i+1,0]-self.matr[i,0])
        y = math.cos(self.matr[i,1])*math.sin(self.matr[i+1,1]) - math.sin(self.matr[i,1])*math.cos(self.matr[i+1,1])*math.cos(self.matr[i+1,0] - self.matr[i,0])
      else:
        x = math.cos(self.target[1])*math.sin(self.target[0]-self.matr[i,0])
        y = math.cos(self.matr[i,1])*math.sin(self.target[1]) - math.sin(self.matr[i,1])*math.cos(self.target[1])*math.cos(self.target[0] - self.matr[i,0])
      self.angles[i] = math.atan2(x,y)

  def getTurn(self):
    self.error = self.angles[3] - self.angles[2]
    if (self.error < 0):
      self.turn = 'right'
    elif (self.error > 0):
      self.turn = 'left'
    else:
      print "The 'angles' array has not been filled yet. The 'getRelBearing' function must be ran first."
      return None
    print "Payload must turn {}, {} degrees".format(self.turn, self.error)

  def getTurb(self):
    imu = navio.mpu9250.MPU9250()
    imu.initialize()
    time.sleep(1)
    timer = time.time()
    angle = 0.0
    while True:
      m9a, m9g, m9m = imu.getMotion9()
      angle += m9g[2]/2
      if (time.time()-timer > 5):
        if (abs(angle) < 30):
          print angle
          return True
        else:
          print angle
          return False
      time.sleep(0.25)
      
  def getChange(self): #Measures angular change over time for trajectory correction
    imu = navio.mpu9250.MPU9250()
    imu.initialize()
    time.sleep(1)
    timer = time.time()
    angle = 0.0
    counter = 0
    try:
      while True:
        m9a, m9g, m9m = imu.getMotion9()
        angle += m9g[2]/2
        if (counter == 4):
          print "Moved ", angle*90/3.14
          counter = 0
          if (angle * self.error > 0 and abs(angle) >= abs(self.error)):
            break
        counter += 1
        time.sleep(0.25)
        if (time.time() - timer > 30):
          print "Timeout limit reached (30s) \nThe payload completed {} of the {} degrees necessary for proper adjustment".format(angle, self.error)
          return False
    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
      print "Trajectory corrected {} degrees {}".format(angle, self.turn)
      print "Done.\nExiting."
    print "Trajectory corrected {} degrees {}".format(angle, self.turn)
    return True

  def whatUp(self): #Update user on course progression in relation to target
    print "Last recorded Reference Longitude: ", math.degrees(self.matr[2,0])
    print "Last recorded Reference Latitude:  ", math.degrees(self.matr[2,1])
    print "Last recorded Current Longitude: ", math.degrees(self.matr[3,0])
    print "Last recorded Current Latitude:  ", math.degrees(self.matr[3,1]), "\n"
    
    print "Kilometers travelled between Reference and Current co-ordinates: ", self.distances[2]
    print "Relative bearing between the two positions: ", self.angles[2], "\n"

    print "Distances in km between current co-ordinates and target co-ordinates: ", self.distances[3]
    print "Relative bearing between the two positions: ", self.angles[3], "\n"
#END OF CLASS

p = BerryBoi()
i = 0

while (i == 0): #Test loop to run on pi
  choice = str(raw_input("Choose a function to run: [getData, getDistance, getRelBearing, getChange, getTurb, whatUp]\nChoice: "))
  if (choice == "getData"):
    p.getData(15)
  elif (choice == "getDistance"):
    p.getDistance()
  elif (choice == "getRelBearing"):
    p.getRelBearing()
  elif (choice == "getTurn"):
    p.getTurn()
  elif (choice == "getTurb"):
    p.getTurb()
  elif (choice == "getChange"):
    p.getChange()
  elif (choice == "whatUp"):
    p.whatUp()
  else:
    print "Unkown string submission"
  
  choice = str(raw_input("Would you like to do something else? (y/n) "))
  if (choice == "y"):
    i = 0
  else:
    i = 1

#MAIN LOOP
print "Entering main loop..."
try:
  while True:
    p.getData()
    p.getDistance()
    p.getRelBearing()
    p.whatUp()
    p.getTurn()
    if (p.getTurb()):
      #adjust payload by 'p.error' degrees and in direction 'p.turn' with servos at this point
      p.getChange() #Monitor turn rate and terminate when turn is completed
    else:
      #choiceThatDoesntMatter = raw_input -> in the future have ground station specify a wait time until resuming navigation and checking for turbulence again
      print "Turbulence is preventing the payload from accurately navigating"
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "Main loop terminated.\nBye!"
