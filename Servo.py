import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setup(32, GPIO.OUT)

pwm = GPIO.PWM(32, 50)
pwm.start(0)

def setAngle(angle):
  duty = angle/18 + 2
  GPIO.output(32, True)
  pwm.ChangeDutyCycle(duty)
  sleep(1)
  GPIO.output(32, False)
  pwm.ChangeDutyCycle(0)
  pwm.stop()
  GPIO.cleanup()

x = True
while (x == True):
  theta = int(raw_input("Enter an angle to turn the servo by: "))
  print "Turning servo {} degrees".format(theta)
  setAngle(theta)
  print "Done."
  choice = raw_input("Would you like to turn the servo again? (y/n): ")
  if (choice !=  'y'):
    x = False
    print "Bye!"
