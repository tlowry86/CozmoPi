import cozmo
import time
import os
import _thread
import math
import RPi.GPIO as GPIO
import signal
import sys
import dht11
import hcsr04
import datetime
import busio # ADC library
import digitalio # ADC library
import board # ADC library
import adafruit_mcp3xxx.mcp3008 as MCP # ADC library
from adafruit_mcp3xxx.analog_in import AnalogIn # ADC library
from PIL import Image
from gpiozero import Button
from signal import pause
from cozmo.util import degrees

button_F = Button(6, hold_time = .15)
button_B = Button(13, hold_time = .15)
button_L = Button(19, hold_time = .15)
button_R = Button(26, hold_time = .15)
button_1 = Button(16, hold_time = .2)
button_2 = Button(20, hold_time = .2)
button_3 = Button(21, hold_time = 1)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup(18)

tempHumSensor = dht11.DHT11(pin=18) # Creating Temperature/Humidity sensor object

distSensor = hcsr04.HCSR04(trig=24, echo=23) # Creating Distance sensor object

execute = False
move = True
cameraActive = False
cameraButton = False

imageNum = 0

maxSpd = 200
currentLiftPos = 0
currentHeadAng = 0
joystickDZ = range(31000,33500)
distanceList = []

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)
mcp = MCP.MCP3008(spi, cs)

LJoyPos = AnalogIn(mcp, MCP.P1)
RJoyPos = AnalogIn(mcp, MCP.P4)
headAng = AnalogIn(mcp, MCP.P6)
liftPos = AnalogIn(mcp, MCP.P7)

def stopCozmo():
        global move

        # Condition prevents execution prior to main function calling it.
        if execute:
                move = False

def moveCozmo():
        global move

        # Condition prevents execution prior to main function calling it.
        if execute:
                move = True

def ImageCapture(robot):
        global imageNum
        
        robot.camera.image_stream_enabled = True
        nImg = robot.world.wait_for(cozmo.world.EvtNewCameraImage)
        nImg.image.raw_image.show()

        Img = robot.world.latest_image.raw_image
        Img = Img.convert('L')
        imageNum = imageNum + 1
        Img.save("Image"+str(imageNum)+".bmp")

        robot.camera.image_stream_enabled = False
        print("Image Capture Complete")

def JoyCalc(LTrac,RTrac):
        # Seperate dictionaries for less sensitivity with joysticks
##        ForwardMoSpds = {range(0,8000):1.0, range(8000,16000):.75, range(16000,24000):.50, range(24000,32001):.25}
##        BackwardMoSpds = {range(57500,65500):1.0, range(49500,57500):.75, range(41500,49500):.50, range(33500,41500):.25}

        # Libraries determining % of speed based on read out from Joystick
        ForwardMoSpds = {range(0,3200):1.0, range(3200,6400):.9, range(6400,9600):.8, range(9600,12800):.7, range(12800,16000):.6,
                         range(16000,19200):.5, range(19200,22400):.4, range(22400,25600):.3, range(25600,28800):.2, range(28800,31000):.1}
        BackwardMoSpds = {range(62300,65500):1.0, range(59100,62300):.9, range(55900,59100):.8, range(52700,55900):.7, range(49500,52700):.6,
                          range(46300,49500):.5, range(43100,46300):.4, range(39900,43100):.3, range(36700,39900):.2, range(33500,36700):.1}
        
        ForwardMo = range(0,31000)
        BackMo = range(33500,65500)
        
        # Convert X value to a value between 0 and 1
        if LTrac in ForwardMo:
                for r in ForwardMoSpds:
                        if LTrac in r:
                                SP = ForwardMoSpds.get(r)
                                break
                LTracSpd = maxSpd*SP
        elif LTrac in BackMo:
                for r in BackwardMoSpds:
                        if LTrac in r:
                                SP = BackwardMoSpds.get(r)
                                break
                LTracSpd = -1*(maxSpd*SP)
        else:
                LTracSpd = 0

        # Convert Y value to a value between 0 and 1
        if RTrac in ForwardMo:
                for r in ForwardMoSpds:
                        if RTrac in r:
                                SP = ForwardMoSpds.get(r)
                                break
                RTracSpd = maxSpd*SP
        elif RTrac in BackMo:
                for r in BackwardMoSpds:
                        if RTrac in r:
                                SP = BackwardMoSpds.get(r)
                                break
                RTracSpd = -1*(maxSpd*SP)
        else:
                RTracSpd = 0

        return (LTracSpd,RTracSpd)

def tempHumChk(robot):
        tempHumData = tempHumSensor.read()
        tempF = round(((tempHumData.temperature * 9/5)+32))
        humidity = round(tempHumData.humidity)

        if tempHumData.is_valid():
                print("Temperature: %d F" % (tempF))
                print("Humidity: %d %%" % humidity)

                if tempF < 40:
                        robot.say_text("The temperature is {} degrees and I'm pretty sure winter is coming.".format(tempF)).wait_for_completed()
                elif tempF > 90:
                        robot.say_text("The temperature is {} and I feel like I'm in a nelly song because it is hot in here.".format(tempF)).wait_for_completed()
                else:
                        robot.say_text("The temperature is {} degrees.".format(tempF)).wait_for_completed()
                        
                if humidity > 70:
                        robot.say_text("Its not the heat its the gosh darn humidity which is {} percent.".format(humidity)).wait_for_completed()
                                      
        time.sleep(1)

def distanceChk(robot):
        global distanceList
        
        distance = distSensor.read()
        if len(distanceList) == 0:
                distanceList.append(distSensor.read())
                print("Initial Distance is : {}".format(distanceList[0]))
                time.sleep(1)
        elif len(distanceList) == 1:
                distanceList.append(distSensor.read())
                print("Final Distance is : {}".format(distanceList[1]))
                distTraveled = distanceList[1] - distanceList[0]
                print("Distance Traveled is : {}".format(distTraveled))

                # Has +/- 2 cm tolerance due to flucuations in readouts.
                if distTraveled > 2:
                        robot.say_text("I moved away from the sensor {} centimeters".format(str(round(distTraveled,1))))
                elif distTraveled < -2:
                        robot.say_text("I moved towards the sensor {} centimeters".format(str(round((distTraveled * -1),1))))
                else:
                        robot.say_text("I didn't move.")
                
                # Resets the list
                distanceList = []
                
def liftPosCheck(liftPos):
        Pos = 0
        positionDict = {range(0,5950):0, range(5950,11900):.1, range(11900,17850):.2, range(17850,23800):.3,
                     range(23800,29750):.4, range(29750,35700):.5, range(35700,41650):.6, range(41650,47600):.7,
                     range(47600,53550):.8, range(53550,59500):.9, range(59500,65500):1.0}
        
        for r in positionDict:
                        if liftPos in r:
                                position = positionDict.get(r)

        return position

def headAngCheck(headAng):
        angle = 0
        angleDict = {range(0,4350):-25.0, range(4350,8700):-20.0, range(8700,13050):-15.0, range(13050,17400):-10.0,
                     range(17400,21750):-5.0, range(21750,26100):0, range(26100,30450):5.0, range(30450,34800):10.0,
                     range(34800,39150):15.0, range(39150,43500):20.0, range(43500,47850):25.0, range(47850,52200):30.0,
                     range(52200,56550):35.0, range(56550,60900):40.0, range(60900,65500):44.5}
        
        for r in angleDict:
                        if headAng in r:
                                angle = angleDict.get(r)

        return angle
	
def cozmo_program(robot: cozmo.robot.Robot):
        global execute
        global move
        global cameraActive
        global cameraButton
        global currentLiftPos
        global currentHeadAng

        execute = True
        run = True
        
        while run:
                # Sets global var move to False when button_2 is released.
                button_F.when_released = stopCozmo
                button_F.when_held = moveCozmo
                button_B.when_released = stopCozmo
                button_B.when_held = moveCozmo
                button_L.when_released = stopCozmo
                button_L.when_held = moveCozmo
                button_R.when_released = stopCozmo
                button_R.when_held = moveCozmo

                # Joystick Controls
                if LJoyPos.value not in joystickDZ or RJoyPos.value not in joystickDZ:
                        
                        while True:
                                ##print("LJoyPos : {} RJoyPos : {}".format(LJoyPos.value,RJoyPos.value))
                                LTracSpd,RTracSpd = JoyCalc(LJoyPos.value,RJoyPos.value)
                                if LTracSpd == 0 and RTracSpd == 0:
                                        break
                                robot.drive_wheel_motors(LTracSpd,RTracSpd)
                                time.sleep(.05)
                        robot.stop_all_motors()

                # Determines Lift Position
                PosLift = liftPosCheck(liftPos.value)
                if PosLift != currentLiftPos:
                        currentLiftPos = PosLift
                        robot.set_lift_height(currentLiftPos).wait_for_completed()
                        print("Lift Position : {}".format(currentLiftPos))

                # Determines Head Angle
                angleHead = headAngCheck(headAng.value)
                if angleHead != currentHeadAng:
                        currentHeadAng = angleHead
                        robot.set_head_angle(degrees(currentHeadAng)).wait_for_completed()
                        print("Head Angle : {}".format(currentHeadAng))
                        
                # Move Forward
                if button_F.is_held == True:
                        
                        while move:
                                robot.drive_wheel_motors(75,75)        
                        robot.stop_all_motors()

                # Move Backwards
                if button_B.is_held == True:
        
                        while move:
                                robot.drive_wheel_motors(-75,-75)        
                        robot.stop_all_motors()

                # Turn Left
                if button_L.is_held == True:
                        
                        while move:
                                robot.drive_wheel_motors(-75,75)        
                        robot.stop_all_motors()

                # Turn Right
                if button_R.is_held == True:
                        
                        while move:
                                robot.drive_wheel_motors(75,-75)        
                        robot.stop_all_motors()

                # Take a Picture on with Button Press and Activate/Deactivate Camera with a button hold.        
                if button_1.is_pressed == True:
                        ctr = 0

                        # Condition to seperate a button press vs button held.
                        while button_1.is_active:
                                if button_1.is_held == True and ctr == 0:
                                        tempHumChk(robot)
                                        ctr = ctr + 1
                        if ctr == 0:
                                distanceChk(robot)
                                
                # Take a picture when button is held down and performs when button is Pressed.
                if button_2.is_pressed == True:
                        ctrCam = 0

                        while button_2.is_active:
                                if button_2.is_held == True and ctrCam == 0:
                                        ImageCapture(robot)
                                        ctrCam = ctrCam + 1
                        if ctrCam == 0:
                                robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabDog).wait_for_completed()
                        
                        
                if button_3.is_pressed == True:
                        ctrExit = 0

                        while button_3.is_active:
                                if button_3.is_held and ctrExit == 0:
                                        print("\nSystem Shutting Down\n")
                                        GPIO.cleanup()
                                        sys.exit(0)
                        if ctrExit == 0:
                                robot.drive_wheel_motors(75,-75)
                                robot.say_text("You spin me right round baby right round.",in_parallel=True).wait_for_completed()
                                robot.stop_all_motors()


cozmo.run_program(cozmo_program)			
