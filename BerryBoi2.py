import math
import time
import inspect
from gps import *

def makeTri(a, b):
  tri = [0,0,0]
  tri[0] = abs(b[0] - a[0])
  tri[1] = abs(b[1] - a[1])
  tri[2] = (tri[0]**2 + tri[1]**2)**0.5
  return tri 

class Flight():
  gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
  GPStime = 0.0
  lat = 0.0
  lon = 0.0
  speed = 0.0
  error = 0.0
  turn = " "
  mag = 0.0
  t = [-122, 36.6]
  a = [-117.6, 24.1]
  b = [-119.2, 26.3]
  #m9a, m9g, m9m = imu.getMotion9()
  
  def getData(self):
    timeyBoi = 0
    countyBoi = 0
    x = False
    try:
      while (x == False):
        report = self.gpsd.next()
        sys.stdout.write('\r'+"Loading 'TPV'")
        time.sleep(0.1)
        sys.stdout.flush()
        sys.stdout.write('\r'+"Loading 'TPV'.")
        time.sleep(0.1) 
        sys.stdout.flush()
        sys.stdout.write('\r'+"Loading 'TPV'..")
        time.sleep(0.1)
        sys.stdout.flush()
        sys.stdout.write('\r'+"Loading 'TPV'...")
        time.sleep(0.1)
          
        if report['class']== 'TPV':
          time.sleep(0.1)
          sys.stdout.write('\r'+"({}s) Searching for GPS data".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          sys.stdout.write('\r'+"({}s) Searching for GPS data.".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          sys.stdout.write('\r'+"({}s) Searching for GPS data..".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          sys.stdout.write('\r'+"({}s) Searching for GPS data...".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          sys.stdout.write('\r'+"({}s) Searching for GPS data..".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          sys.stdout.write('\r'+"({}s) Searching for GPS data.".format(timeyBoi))
          time.sleep(0.1)
          timeyBoi+=.1
          sys.stdout.flush()
          self.GPStime =  getattr(report,'time','')
          self.lat = getattr(report,'lat',0.0)
          self.lon = getattr(report,'lon',0.0)
          self.speed =  getattr(report,'speed','nan')
          if (countyBoi == 1 and self.lat != 0.0 and self.lon != 0.0):
            self.b[0] = self.lon
            self.b[1] = self.lat
            print "Current longitude: ", self.lon
            print "Current latitude: ", self.lat
            print "Speed: ", self.speed
            print "Time: ", self.GPStime
            x = True
          if (self.lat != 0.0 and self.lon != 0.0 and x == False):
            self.a[0] = self.lon
            self.a[1] = self.lat
            countyBoi += 1
            print "Reference longitude: ", self.lon
            print "Reference latitude: ", self.lat
            print "Time: ", self.GPStime, "\n"
            print "Waiting 60 seconds to collect more data...\n"
            time.sleep(60)
          
          """
          print  GPStime,"\t",
          print  lat,"\t",
          print  lon,"\t",
          print  speed,"\t",
          print  sats,"\t"
          """ 
          #time.sleep(1)
    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
      print "Done.\nExiting."

  def error(self):
    refTri = makeTri(self.a, self.b)
    targTri = makeTri(self.b, self.t)
    refTheta = 180*math.acos(refTri[0]/refTri[2])/math.pi
    targTheta = 180*math.acos(targTri[0]/targTri[2])/math.pi
    if (refTheta < 0):
      refTheta += 360
    if (targTheta < 0):
      targTheta += 360
    self.error = refTheta - targTheta
    print "Angle of adjustment: {}".format(self.error)
    #return self.error*180/math.pi

  def turn(self):
    slope = (self.b[1]-self.a[1])/(self.b[0]-self.a[0])
    #print slope
    x_val = self.t[0]-self.b[0]
    #print x_val
    y_val = slope * x_val + self.b[1]
    #print y_val
    #print self.t[1]
    if ((self.b[0] - self.a[0]) > 0):
      if (y_val - self.t[1] < 0):
        self.turn = "left"
      else:
        self.turn = "right"
    else:
      if (y_val - self.t[1] < 0):
        self.turn = "right"
      else:
        self.turn = "left"
    print "Preparing to turn " + self.turn
  
  def getArc(self):
    x = ((self.b[0]-self.a[0])**2 + (self.b[1]-self.a[1])**2)**0.5
    print x
    self.theta = math.acos((x**2-2(r_earth**2))/(-2(r_earth**2)))
    print "Payload is {} feet from target position".format(theta*r_earth)
  

r_earth = 20902464.0
p = Flight()
i = 0

def custom():
  #custom = raw_input("Would you like to enter custom values for positions 'a', 'b', and 't'? (y/n)")
  #if (custom == 'y'):
    ax = float(raw_input('Reference position longitude: '))
    ay = float(raw_input('Reference position latitude: '))
    bx = float(raw_input('Current position longitude: '))
    by = float(raw_input('Current position latitude: '))
    tx = float(raw_input('Target position longitude: '))
    ty = float(raw_input('Target position latitude: '))
    p.a = [ax, ay]
    p.b = [bx, by]
    p.t = [tx, ty]

while (i == 0):
  choice = str(raw_input("Choose sum to run ya herrrd (getData, getArc, checkSafe, error, turn, custom): "))
  if (choice == "getData"):
    p.getData()
  elif (choice == "getArc"):
    p.getArc()
  elif (choice == "checkSafe"):
    p.checkSafe()
  elif (choice == "error"):
    p.error()
  elif (choice == "turn"):
    p.turn()
  elif (choice == "custom"):
    custom()
  else:
    print "Unkown string submission dumbo"

  choice = str(raw_input("Wanna do sumthin else? (y/n) "))
  if (choice == "y"):
    i = 0
  else:
    i = 1
