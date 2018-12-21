#
# This file handles GoPro commands and holds GoPro state
#
import os
import Queue
import sys
import threading
import time
import monotonic
import math
import signal
from pymavlink import mavutil

sys.path.append(os.path.realpath(''))
import app_packet
from GoProConstants import *
import settings
import shotLogger
import struct

# Pymata import
from PyMata.pymata import PyMata

logger = shotLogger.logger
# Gopro Mod
SOLO_MOD = "GOPRO"
GOPRO_FLUSH = 7
OUTPUT1 = 8
OUTPUT2 = 9
HANDSHAKE = 10
PYMATA_STATUS = 11

# tuple of message types that we handle
GOPROMESSAGES = \
(
    app_packet.GOPRO_SET_ENABLED,
    app_packet.GOPRO_SET_REQUEST,
    app_packet.GOPRO_RECORD,
    app_packet.GOPRO_REQUEST_STATE,
    app_packet.GOPRO_SET_EXTENDED_REQUEST
)

# see https://docs.google.com/document/d/1CcYOCZRw9C4sIQu4xDXjPMkxZYROmTLB0EtpZamnq74/edit#heading=h.y6z65lvic5q5
VALID_GET_COMMANDS = \
(
    mavutil.mavlink.GOPRO_COMMAND_POWER,
    mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE,
    mavutil.mavlink.GOPRO_COMMAND_BATTERY,
    mavutil.mavlink.GOPRO_COMMAND_MODEL,
    mavutil.mavlink.GOPRO_COMMAND_VIDEO_SETTINGS,
    mavutil.mavlink.GOPRO_COMMAND_LOW_LIGHT,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_RESOLUTION,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_BURST_RATE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_COLOUR,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_GAIN,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_SHARPNESS,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_EXPOSURE,
    mavutil.mavlink.GOPRO_COMMAND_TIME,
    mavutil.mavlink.GOPRO_COMMAND_CHARGING,
)

VALID_SET_COMMANDS = \
(
    mavutil.mavlink.GOPRO_COMMAND_POWER,
    mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE,
    mavutil.mavlink.GOPRO_COMMAND_SHUTTER,
    mavutil.mavlink.GOPRO_COMMAND_VIDEO_SETTINGS,
    mavutil.mavlink.GOPRO_COMMAND_LOW_LIGHT,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_RESOLUTION,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_BURST_RATE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_COLOUR,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_GAIN,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_SHARPNESS,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_EXPOSURE,
    mavutil.mavlink.GOPRO_COMMAND_TIME,
    mavutil.mavlink.GOPRO_COMMAND_CHARGING,
)

REQUERY_COMMANDS = \
(
    mavutil.mavlink.GOPRO_COMMAND_VIDEO_SETTINGS,
    mavutil.mavlink.GOPRO_COMMAND_LOW_LIGHT,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_RESOLUTION,
    mavutil.mavlink.GOPRO_COMMAND_PHOTO_BURST_RATE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_COLOUR,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_GAIN,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_SHARPNESS,
    mavutil.mavlink.GOPRO_COMMAND_PROTUNE_EXPOSURE,
    mavutil.mavlink.GOPRO_COMMAND_TIME,
)

class GoProManager():
    def __init__(self, shotMgr):
        # GoPro heartbeat state
        if SOLO_MOD == "GOPRO":    
            try:
                logger.log("[gopromanager-arduino]: try to open Arduino PyMata")
                self.arduinoBoard = PyMata("/dev/ttyACM0", verbose=True)
                logger.log("[gopromanager-arduino]: Arduino PyMata OPENED - OK")
                self.arduinoBoard.set_pin_mode(OUTPUT1, self.arduinoBoard.INPUT, self.arduinoBoard.DIGITAL)
                self.arduinoBoard.set_pin_mode(OUTPUT2, self.arduinoBoard.INPUT, self.arduinoBoard.DIGITAL)
                # register callback
                logger.log("[gopromanager-arduino]: try to register callback")
                self.arduinoBoard.set_pin_mode(HANDSHAKE, self.arduinoBoard.INPUT, self.arduinoBoard.DIGITAL, self.goproModelType)
                logger.log("[gopromanager-arduino]: Arduino PyMata callback registered")  
                # tell the Arduino we are ready
                self.arduinoBoard.digital_write(PYMATA_STATUS, 1)
            except:
                logger.log("[gopromanager-arduino]: Error in communication to Arduino")

            self.status = 0
            self.model = MODEL_NONE
            # enter here the default settings of your Gopro
            self.captureMode = CAPTURE_MODE_VIDEO
            self.isRecording = False
            self.battery = 1
            self.videoFormat = VIDEO_FORMAT_PAL
            self.videoResolution = 11
            self.videoFrameRate = 6
            self.videoFieldOfView = 2
            self.videoLowLight = True
            self.photoResolution = 4
            self.photoBurstRate = 1
            self.videoProtune = True
            self.videoProtuneWhiteBalance = 0
            self.videoProtuneColor = 0
            self.videoProtuneGain = 0
            self.videoProtuneSharpness = 0
            self.videoProtuneExposure = 10
            
        else:    
            self.status = mavutil.mavlink.GOPRO_HEARTBEAT_STATUS_DISCONNECTED
            self.model = MODEL_NONE
            self.captureMode = CAPTURE_MODE_VIDEO
            self.isRecording = False
            # Additional GoPro state
            self.battery = 0
            self.videoFormat = VIDEO_FORMAT_NTSC
            self.videoResolution = 0
            self.videoFrameRate = 0
            self.videoFieldOfView = 0
            self.videoLowLight = False
            self.photoResolution = 0
            self.photoBurstRate = 0
            self.videoProtune = False
            self.videoProtuneWhiteBalance = 0
            self.videoProtuneColor = 0
            self.videoProtuneGain = 0
            self.videoProtuneSharpness = 0
            self.videoProtuneExposure = 0

        self.shotMgr = shotMgr
        # This exists because we can't seem to send multiple messages in a stream to the gopro.
        # Instead, we'll queue up all our messages and wait for a response before sending the next message
        self.msgQueue = Queue.Queue()
        # is the GoPro currently handling a message?
        self.isGoproBusy = False
        # when the last message was sent
        self.lastRequestSent = 0.0
        # lock access to shot manager state
        self.lock = threading.Lock()

        # check if we should enable GoPro messages at all
        try:
            enabled = int(settings.readSetting("GoProEnabled"))
            logger.log("[gopro]: read enabled value from settings of %d."%(enabled))
            self.enabled = enabled > 0
            self.setGimbalEnabledParam()
        except Exception as ex:
            logger.log("[gopro]: Error reading config file.")
            logger.log(str(ex))
            self.enabled = True

        logger.log("[gopro]: Inited GoProManager")

    def state_callback(self, vehicle, name, message):
        self.lock.acquire()
        try:
            self.internal_state_callback(message)
        except Exception as e:
            logger.log("[gopro]: state_callback error: %s" % e)
        finally:
            self.lock.release()

    def internal_state_callback(self, state):
        status = state.status
        captureMode = state.capture_mode
        isRecording = (state.flags & mavutil.mavlink.GOPRO_FLAG_RECORDING) != 0
        sendState = False
        logger.log("[gopro-test]: state_callback %s" % status)
        if self.status != status:
            self.status = status
            logger.log("[gopro]: Gopro status changed to %d"%(self.status))
            sendState = True

            # right now, query status when we initially connect
            if self.status == mavutil.mavlink.GOPRO_HEARTBEAT_STATUS_CONNECTED:
                self.isGoproBusy = False
                self.msgQueue = Queue.Queue()
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_BATTERY)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_MODEL)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_VIDEO_SETTINGS)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_LOW_LIGHT)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PHOTO_RESOLUTION)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PHOTO_BURST_RATE)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE_COLOUR)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE_GAIN)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE_SHARPNESS)
                self.sendGoProRequest(mavutil.mavlink.GOPRO_COMMAND_PROTUNE_EXPOSURE)

        if self.captureMode != captureMode:
            self.captureMode = captureMode
            logger.log("[gopro]: Gopro capture mode changed to %d"%(self.captureMode))
            sendState = True

        if self.isRecording != isRecording:
            self.isRecording = isRecording
            logger.log("[gopro]: Gopro recording status changed to %d"%(self.isRecording))
            sendState = True

        if sendState:
            self.sendState()

    def get_response_callback(self, vehicle, name, message):
        self.lock.acquire()
        try:
            self.internal_get_response_callback(message)
        except Exception as e:
            logger.log("[gopro]: get_response_callback error: %s" % e)
        finally:
            self.lock.release()

    def internal_get_response_callback(self, response):
        command = response[0]
        status = response[1]
        value = response[2]
        sendState = False

        if status != mavutil.mavlink.GOPRO_REQUEST_SUCCESS:
            logger.log("[gopro]: Gopro get request for command %d failed with status %d"%(command, status))
            self.processMsgQueue()
            return

        if command == mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE:
            captureMode = value[0]
            if self.captureMode != captureMode:
                self.captureMode = captureMode
                sendState = True
                logger.log("[gopro]: Gopro capture mode changed to %d"%(self.captureMode))
        elif command == mavutil.mavlink.GOPRO_COMMAND_MODEL:
            model = value[0]
            if self.model != model:
                self.model = model
                sendState = True
                logger.log("[gopro]: Gopro model changed to %d"%(self.model))
        elif command == mavutil.mavlink.GOPRO_COMMAND_BATTERY:
            battery = value[0]
            if self.battery != battery:
                self.battery = battery
                sendState = True
                logger.log("[gopro]: Gopro battery changed to %d"%(self.battery))
        elif command == mavutil.mavlink.GOPRO_COMMAND_VIDEO_SETTINGS:
            videoResolution = value[0]
            videoFrameRate = value[1]
            videoFieldOfView = value[2]
            videoFormat = VIDEO_FORMAT_NTSC if (value[3] & mavutil.mavlink.GOPRO_VIDEO_SETTINGS_TV_MODE) == 0 else VIDEO_FORMAT_PAL
            if self.videoResolution != videoResolution:
                self.videoResolution = videoResolution
                sendState = True
                logger.log("[gopro]: Gopro video resolution changed to %d"%(self.videoResolution))
            if self.videoFrameRate != videoFrameRate:
                self.videoFrameRate = videoFrameRate
                sendState = True
                logger.log("[gopro]: Gopro video frame rate changed to %d"%(self.videoFrameRate))
            if self.videoFieldOfView != videoFieldOfView:
                self.videoFieldOfView = videoFieldOfView
                sendState = True
                logger.log("[gopro]: Gopro video field of view changed to %d"%(self.videoFieldOfView))
            if self.videoFormat != videoFormat:
                self.videoFormat = videoFormat
                sendState = True
                logger.log("[gopro]: Gopro video format changed to %d"%(self.videoFormat))
        elif command == mavutil.mavlink.GOPRO_COMMAND_LOW_LIGHT:
            videoLowLight = value[0] != 0
            if self.videoLowLight != videoLowLight:
                self.videoLowLight = videoLowLight
                sendState = True
                logger.log("[gopro]: Gopro low light changed to %d"%(self.videoLowLight))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PHOTO_RESOLUTION:
            photoResolution = value[0]
            if self.photoResolution != photoResolution:
                self.photoResolution = photoResolution
                sendState = True
                logger.log("[gopro]: Gopro photo resolution changed to %d"%(self.photoResolution))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PHOTO_BURST_RATE:
            photoBurstRate = value[0]
            if self.photoBurstRate != photoBurstRate:
                self.photoBurstRate = photoBurstRate
                sendState = True
                logger.log("[gopro]: Gopro photo burst rate changed to %d"%(self.photoBurstRate))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE:
            videoProtune = value[0] != 0
            if self.videoProtune != videoProtune:
                self.videoProtune = videoProtune
                sendState = True
                logger.log("[gopro]: Gopro video protune changed to %d"%(self.videoProtune))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE:
            videoProtuneWhiteBalance = value[0]
            if self.videoProtuneWhiteBalance != videoProtuneWhiteBalance:
                self.videoProtuneWhiteBalance = videoProtuneWhiteBalance
                sendState = True
                logger.log("[gopro]: Gopro video protune white balance changed to %d"%(self.videoProtuneWhiteBalance))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE_COLOUR:
            videoProtuneColor = value[0]
            if self.videoProtuneColor != videoProtuneColor:
                self.videoProtuneColor = videoProtuneColor
                sendState = True
                logger.log("[gopro]: Gopro video protune color changed to %d"%(self.videoProtuneColor))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE_GAIN:
            videoProtuneGain = value[0]
            if self.videoProtuneGain != videoProtuneGain:
                self.videoProtuneGain = videoProtuneGain
                sendState = True
                logger.log("[gopro]: Gopro video protune gain changed to %d"%(self.videoProtuneGain))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE_SHARPNESS:
            videoProtuneSharpness = value[0]
            if self.videoProtuneSharpness != videoProtuneSharpness:
                self.videoProtuneSharpness = videoProtuneSharpness
                sendState = True
                logger.log("[gopro]: Gopro video protune sharpness changed to %d"%(self.videoProtuneSharpness))
        elif command == mavutil.mavlink.GOPRO_COMMAND_PROTUNE_EXPOSURE:
            videoProtuneExposure = value[0]
            if self.videoProtuneExposure != videoProtuneExposure:
                self.videoProtuneExposure = videoProtuneExposure
                sendState = True
                logger.log("[gopro]: Gopro video protune exposure changed to %d"%(self.videoProtuneExposure))
        else:
            logger.log("[gopro]: Got unexpected Gopro callback for command %d"%(command))

        if sendState:
            self.sendState()

        self.processMsgQueue()

    def set_response_callback(self, vehicle, name, message):
        self.lock.acquire()
        try:
            self.internal_set_response_callback(message)
        except Exception as e:
            logger.log("[gopro]: set_response_callback error: %s" % e)
        finally:
            self.lock.release()

    def internal_set_response_callback(self, response):
        command = response[0]
        status = response[1]

        logger.log("[gopro]: Got Gopro set response for command %d with status %d"%(command, status))
        if status != mavutil.mavlink.GOPRO_REQUEST_SUCCESS:
            # if a set request failed, return current state to resynchronize client
            self.sendState()

        self.processMsgQueue()

    # wrapper to create a gopro_get_request mavlink message with the given command
    def sendGoProRequest(self, command):
        if command not in VALID_GET_COMMANDS:
            logger.log("[gopro]: Got invalid Gopro get command %d"%(command))
            return

        msg = self.shotMgr.vehicle.message_factory.gopro_get_request_encode(
                                    0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,    # target system, target component
                                    command
                                    )

        self.queueMsg(msg)

    # wrapper to create a gopro_set_request mavlink message with the given command and value
    def sendGoProCommand(self, command, value):
        if command not in VALID_SET_COMMANDS:
            logger.log("[gopro]: Got invalid Gopro set command %d"%(command))
            return
        if not self.isValidCommandValue(value):
            logger.log("[gopro]: Invalid value for Gopro set command %d"%(command))
            return

        if SOLO_MOD == "GOPRO":
            sendState = False
            #directly send the corresponding Gopro commands
            
            if command == 0 and (value[0] == 0 or value [0] == 1):   
                self.setGoProCommandArduino(value[0]) #toggle vide recording on/off 
                if self.isRecording == 0 and self.captureMode == 0 and value [0] == 1:
                    self.isRecording = 1
                else:
                    self.isRecording = 0
                sendState = True
            if command == 1: # set capture mode
                captureMode = value[0]
                if self.captureMode != captureMode:
                    self.captureMode = captureMode
                    if value[0] == 0:
                        self.setGoProCommandArduino(4)  # set video mode
                        self.setGoProCommandArduino(89)  # set submode video
                    if value[0] == 1:
                        self.setGoProCommandArduino(5)  # set photo mode
                        self.setGoProCommandArduino(93)  # set submode single photo
                    if value[0] == 4:
                        self.setGoProCommandArduino(6)  # set photo burst mode
                        self.setGoProCommandArduino(96)  # set submode burst
                    if value[0] == 5:
                        self.setGoProCommandArduino(4)  # set video mode
                        self.setGoProCommandArduino(110)  # set video timewarp submode
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro capture mode changed to %d"%(self.captureMode))
                    
            if command == 5: # set video modes
                videoResolution = value[0]
                videoFrameRate = value[1]
                videoFieldOfView = value[2]
                videoFormat = VIDEO_FORMAT_NTSC if (value[3]) == 0 else VIDEO_FORMAT_PAL
                if self.videoResolution != videoResolution:
                    self.videoResolution = videoResolution
                    if (value[0] == 0 and (self.model == MODEL_HERO4_BLACK or self.model == MODEL_HERO5_BLACK)):
                        self.setGoProCommandArduino(45)  # set video resolution 480
                    if value[0] == 1:
                        self.setGoProCommandArduino(44)  # set video resolution 720
                    if (value[0] == 2 and (self.model != MODEL_HERO6_BLACK)):
                        self.setGoProCommandArduino(42)  # set video resolution 960
                    if value[0] == 3:
                        self.setGoProCommandArduino(41)  # set video resolution 1080
                    if value[0] == 4:
                        self.setGoProCommandArduino(39)  # set video resolution 1440
                    if value[0] == 5:
                        self.setGoProCommandArduino(39)  # set video resolution not defined
                    if value[0] == 6:
                        self.setGoProCommandArduino(36)  # set video resolution 2,7k
                    if value[0] == 8:
                        self.setGoProCommandArduino(34)  # set video resolution 4k
                    if (value[0] == 10 and (self.model == MODEL_HERO4_BLACK or self.model == MODEL_HERO5_BLACK)):
                        self.setGoProCommandArduino(43)  # set video resolution 720superwide
                    if value[0] == 11:
                        self.setGoProCommandArduino(40)  # set video resolution 1080p superview
                    if value[0] == 12:
                        self.setGoProCommandArduino(37)  # set video resolution 2,7 superview
                    if value[0] == 13:
                        self.setGoProCommandArduino(35)  # set video resolution 4k superview
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video resolution changed to %d"%(self.videoResolution))
                    
                if self.videoFrameRate != videoFrameRate:
                    self.videoFrameRate = videoFrameRate
                    if value[1] == 2:
                        self.setGoProCommandArduino(56)  # set video framerate 24fps   
                    if value[1] == 3:
                        self.setGoProCommandArduino(55)  # set video framerate 25fps   
                    if value[1] == 4:
                        self.setGoProCommandArduino(54)  # set video framerate 30fps   
                    if value[1] == 5:
                        self.setGoProCommandArduino(53)  # set video framerate 48fps   
                    if value[1] == 6:
                        self.setGoProCommandArduino(52)  # set video framerate 50fps   
                    if value[1] == 7:
                        self.setGoProCommandArduino(51)  # set video framerate 60fps   
                    if value[1] == 8:
                        self.setGoProCommandArduino(50)  # set video framerate 80fps   
                    if value[1] == 9:
                        self.setGoProCommandArduino(49)  # set video framerate 90fps   
                    if value[1] == 10:
                        self.setGoProCommandArduino(48)  # set video framerate 100fps   
                    if value[1] == 11:
                        self.setGoProCommandArduino(47)  # set video framerate 120fps   
                    if value[1] == 12:
                        self.setGoProCommandArduino(46)  # set video framerate 240fps   
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video frame rate changed to %d"%(self.videoFrameRate))

                if self.videoFieldOfView != videoFieldOfView:
                    self.videoFieldOfView = videoFieldOfView
                    if value[2] == 0:
                        self.setGoProCommandArduino(57)  # set video fov wide
                    if (value[2] == 1 and (self.model == MODEL_HERO4_BLACK or self.model == MODEL_HERO5_BLACK)):
                        self.setGoProCommandArduino(58)  # set video fov medium
                    if (value[2] == 2 and (self.model == MODEL_HERO4_BLACK or self.model == MODEL_HERO5_BLACK)):
                        self.setGoProCommandArduino(59)  # set video fov Narrow
                    if (value[2] == 3 and (self.model != MODEL_HERO4_BLACK)):
                        self.setGoProCommandArduino(61)  # set video fov LINEAR corresponds to Solex Narrow
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video field of view changed to %d"%(self.videoFieldOfView))
        
                if self.videoFormat != videoFormat:
                    self.videoFormat = videoFormat
                    if value[3] == 0:
                        self.setGoProCommandArduino(81)  # set video NTSC
                    if value[3] == 1:
                        self.setGoProCommandArduino(82)  # set video PAL
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video format changed to %d"%(self.videoFormat))
  
            if command == 6: # set lowlight on off
                videoLowLight = value[0] 
                if self.videoLowLight != videoLowLight:
                    self.videoLowLight = videoLowLight
                    if value[0] == 0:
                        self.setGoProCommandArduino(63)  # set low light off
                    if value[0] == 1:
                        self.setGoProCommandArduino(62)  # set low light on
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro low light changed to %d"%(self.videoLowLight))
                    
            if command == 9: # set video protune on off
                videoProtune = value[0]
                if self.videoProtune != videoProtune:
                    self.videoProtune = videoProtune
                    if value[0] == 0:
                        self.setGoProCommandArduino(84)  # set protune off
                    if value[0] == 1:
                        self.setGoProCommandArduino(83)  # set protune on   
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video protune changed to %d"%(self.videoProtune))
            
            if command == 18: # set video stabilization on off
                videoEIS = value[0]
                if self.videoEIS != videoEIS:
                    self.videoEIS = videoEIS
                    if value[0] == 0:
                        self.setGoProCommandArduino(100)  # set EIS off
                    if value[0] == 1:
                        self.setGoProCommandArduino(99)  # set EIS on   
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video EIS changed to %d"%(self.videoEIS))
            
            if command == 20: # set video time warp speed
                videoWARP = value[0]
                if self.videoWARP != videoWARP:
                    self.videoWARP = videoWARP
                    if value[0] == 1:
                        self.setGoProCommandArduino(111)  # set timewarp speed 2
                    if value[0] == 2:
                        self.setGoProCommandArduino(112)  # set timewarp speed 5   
                    if value[0] == 3:
                        self.setGoProCommandArduino(113)  # set timewarp speed 10 
                    if value[0] == 4:
                        self.setGoProCommandArduino(114)  # set timewarp speed 15  
                    if value[0] == 5:
                        self.setGoProCommandArduino(115)  # set timewarp speed 30  
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro video timewarp speed changed to %d"%(self.videoWARP))
                    
            if command == 7: # photo resolution settings
                photoResolution = value[0]
                if self.photoResolution != photoResolution:
                    self.photoResolution = photoResolution
                    if (value[0] == 0 and self.model == MODEL_HERO4_BLACK):
                        self.setGoProCommandArduino(67)  # set res to 5MP
                    if (value[0] == 1 and self.model == MODEL_HERO4_BLACK):
                        self.setGoProCommandArduino(66)  # set res to 7MP medium
                    if (value[0] == 2 and self.model == MODEL_HERO4_BLACK):
                        self.setGoProCommandArduino(65)  # set res to 7MP wide
                    if value[0] == 4:
                        self.setGoProCommandArduino(64)  # set res to 12MP wide                 
                    if value[0] == 5:
                        self.setGoProCommandArduino(101)  # set res to 12MP linear        
                    if value[0] == 6:
                        self.setGoProCommandArduino(102)  # set res to 12MP medium        
                    if value[0] == 7:
                        self.setGoProCommandArduino(103)  # set res to 12MP narrow        
                    sendState = True
                    logger.log("[gopro]: Gopro photo resolution changed to %d"%(self.photoResolution))

            if command == 16: # set superphoto mode for Hero 7 black only 
                superPhoto = value[0]
                if self.superPhoto != superPhoto:
                    self.superPhoto = superPhoto
                    if value[0] == 0:
                        self.setGoProCommandArduino(104)  # set superphoto off
                    if value[0] == 1:
                        self.setGoProCommandArduino(105)  # set superphoto on   
                    if value[0] == 2:
                        self.setGoProCommandArduino(106)  # set HDR only 
                    sendState = True
                    logger.log("[gopro-arduino]: Gopro superphoto changed to %d"%(self.superPhoto))
                    
            if command == 8: # burst rate settings
                photoBurstRate = value[0]
                if self.photoBurstRate != photoBurstRate:
                    self.photoBurstRate = photoBurstRate
                    if value[0] == 0:
                        self.setGoProCommandArduino(72)  # set burst 3/1
                    if value[0] == 1:
                        self.setGoProCommandArduino(73)  # set burst 5/1
                    if value[0] == 2:
                        self.setGoProCommandArduino(74)  # set burst 10/1
                    if value[0] == 3:
                        self.setGoProCommandArduino(75)  # set burst 10/2
                    if value[0] == 4:
                        self.setGoProCommandArduino(76)  # set burst 10/3
                    if value[0] == 5:
                        self.setGoProCommandArduino(77)  # set burst 30/1
                    if value[0] == 6:
                        self.setGoProCommandArduino(78)  # set burst 30/2
                    if value[0] == 7:
                        self.setGoProCommandArduino(79)  # set burst 30/3
                    if value[0] == 8:
                        self.setGoProCommandArduino(80)  # set burst 30/6  
                    sendState = True
                    logger.log("[gopro]: Gopro photo burst rate changed to %d"%(self.photoBurstRate))   
            
            if command == 14: # video exposure settings 
                videoProtuneExposure = value[0]
                if self.videoProtuneExposure != videoProtuneExposure:
                    self.videoProtuneExposure = videoProtuneExposure
                    if value[0] == 6:
                        self.setGoProCommandArduino(15)  # set video exposure -2
                    if value[0] == 7:
                        self.setGoProCommandArduino(14)  # set video exposure -1,5
                    if value[0] == 8:
                        self.setGoProCommandArduino(13)  # set video exposure -1
                    if value[0] == 9:
                        self.setGoProCommandArduino(12)  # set video exposure -0,5
                    if value[0] == 10:
                        self.setGoProCommandArduino(11)  # set video exposure 0
                    if value[0] == 11:
                        self.setGoProCommandArduino(10)  # set video exposure 0,5
                    if value[0] == 12:
                        self.setGoProCommandArduino(9)  # set video exposure 1
                    if value[0] == 13:
                        self.setGoProCommandArduino(8)  # set video exposure 1,5
                    if value[0] == 14:
                        self.setGoProCommandArduino(7)  # set video exposure 2
                    sendState = True
                    logger.log("[gopro]: Gopro video protune exposure changed to %d"%(self.videoProtuneExposure))
                
            if sendState:
                self.sendState()
 #########################################################
                
        else:
            # execute the standard code
            msg = self.shotMgr.vehicle.message_factory.gopro_set_request_encode(
                                        0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,    # target system, target component
                                        command, value
                                        )

            self.queueMsg(msg)

        if self.captureMode == CAPTURE_MODE_PHOTO:
            if command == mavutil.mavlink.GOPRO_COMMAND_SHUTTER:
                if value[0] == 1:
                    self.sendPhotoEvent()

        # Follow up with a get request if notification of change is required
        if command in REQUERY_COMMANDS:
            self.sendGoProRequest(command)

    # Updates the gopro time.
    def update_gopro_time(self, timeInSecs):
        logger.log("[gopro]: Updating gopro time to " + str(timeInSecs))
        tm = int(timeInSecs)
        tm_list = [tm & 0xff, (tm >> 8) & 0xff, (tm >> 16) & 0xff, (tm >> 24) & 0xff]

        self.sendGoProCommand(mavutil.mavlink.GOPRO_COMMAND_TIME, tm_list)

    def isValidCommandValue(self, value):
        if value[0] < 0 or value[0] > 255:
            return False
        if value[1] < 0 or value[1] > 255:
            return False
        if value[2] < 0 or value[2] > 255:
            return  False
        if value[3] < 0 or value[3] > 255:
            return False
        return True

    # handle a call to start/stop/toggle recording on the given mode
    def handleRecordCommand(self, mode, command):
        #hack to test the wifi communication
        
        if (self.status == mavutil.mavlink.GOPRO_HEARTBEAT_STATUS_DISCONNECTED or self.status == mavutil.mavlink.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE) and SOLO_MOD != "GOPRO":
            logger.log("[gopro]: handleRecordCommand called but GoPro is not connected")
            return

        logger.log("[gopro]: handleRecordCommand called with mode %d, command %d"%(mode, command))
        if SOLO_MOD == "GOPRO":
            cmd1 = 1
            cmd2 = 0
        else:
            cmd1 = mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE
            cmd2 = mavutil.mavlink.GOPRO_COMMAND_SHUTTER

        if mode == CAPTURE_MODE_VIDEO:
            # do we want to start or stop recording?
            if command == RECORD_COMMAND_STOP:
                startstop = 0
            elif command == RECORD_COMMAND_START:
                startstop = 1
            elif command == RECORD_COMMAND_TOGGLE:
                startstop = 0 if self.isRecording else 1
            else:
                return

            # don't start recording if we're already recording
            if self.isRecording and startstop == 1:
                return

            logger.log("[gopro]: Sending command for video recording: %d"%(startstop))

            # we're not in video mode, switch to it!
            if self.captureMode != CAPTURE_MODE_VIDEO:
                self.sendGoProCommand(cmd1, (CAPTURE_MODE_VIDEO, 0 ,0 , 0))
            
            self.sendGoProCommand(cmd2, (startstop, 0, 0, 0))
                    
        elif mode == CAPTURE_MODE_PHOTO or mode == CAPTURE_MODE_BURST or mode == CAPTURE_MODE_MULTISHOT:
            # don't handle nonsensical commands
            if command == RECORD_COMMAND_STOP:
                return

            # for now, let's try switching out of video mode, taking a picture, and then switching back
            if self.captureMode == CAPTURE_MODE_VIDEO:
                self.sendGoProCommand(cmd1, (CAPTURE_MODE_PHOTO, 0, 0, 0))
                self.sendGoProCommand(cmd2, (1, 0, 0, 0))
                self.sendGoProCommand(cmd1, (CAPTURE_MODE_VIDEO, 0, 0, 0))
                logger.log("[gopro]: Sending command to take go to photo mode, take a still, and return")
            else:
                self.sendGoProCommand(cmd2, (1, 0, 0, 0))
                logger.log("[gopro]: Sending command to take a still/burst/multishot")

    # since the gopro can't handle multiple messages at once, we wait for a response before sending
    # each subsequent message.  This is how we queue up messages
    def queueMsg(self, msg):
        if self.isGoproBusy and monotonic.monotonic() > self.lastRequestSent + 2.0:
            self.isGoproBusy = False
            self.msgQueue = Queue.Queue()
            # return current state to resynchronize client
            self.sendState()

        if self.isGoproBusy:
            self.msgQueue.put(msg)
        else:
            self.isGoproBusy = True
            self.lastRequestSent = monotonic.monotonic()
            # Need to send False for fix_targeting so our message gets routed to the gimbal
            self.shotMgr.vehicle.send_mavlink(msg)

    # called whenever the message queue is ready to send another message.
    def processMsgQueue(self):
        if self.msgQueue.empty():
            self.isGoproBusy = False
        else:
            msg = self.msgQueue.get_nowait()
            self.lastRequestSent = monotonic.monotonic()
            # Need to send False for fix_targeting so our message gets routed to the gimbal
            self.shotMgr.vehicle.send_mavlink(msg)
            logger.log("[gopro]: sending message from the queue.  Size is now %d"%(self.msgQueue.qsize()))

    # shotManager receives this gopro control packet and passes it here
    def handlePacket(self, type, data):
        self.lock.acquire()
        try:
            self.internalHandlePacket(type, data)
        finally:
            self.lock.release()

    def internalHandlePacket(self, type, data):
        if type == app_packet.GOPRO_SET_ENABLED:
            (enabled, ) = struct.unpack('<I', data)
            logger.log("[gopro-test]: app.packet.goprosetenabled %d"%(enabled))
            self.setGoProEnabled(enabled > 0)
        elif type == app_packet.GOPRO_SET_REQUEST:
            (command, value) = struct.unpack('<HH', data)
            logger.log("[gopro-test]: app.packet.goprosetrequest-command %d"%(command))
            logger.log("[gopro-test]: app.packet.goprosetrequest-value %d"%(value))
            self.sendGoProCommand(command, (value, 0, 0, 0))
        elif type == app_packet.GOPRO_RECORD:
            (startstop, ) = struct.unpack('<I', data)
            logger.log("[gopro-test]: app.packet.goprorecord-capruremode %d"%(self.captureMode))
            logger.log("[gopro-test]: app.packet.goprorecord-startstop %d"%(startstop))
            self.handleRecordCommand(self.captureMode, startstop)
        elif type == app_packet.GOPRO_REQUEST_STATE:
            logger.log("[gopro-test]: app.packet.goprorequeststate")
            if (SOLO_MOD == "GOPRO"):
                chk = [1,1,1] 
                self.goproModelType(chk)
            else:
                self.sendState()
        elif type == app_packet.GOPRO_SET_EXTENDED_REQUEST:
            (command, value1, value2, value3, value4, ) = struct.unpack("<HBBBB", data)
            logger.log("[gopro-test]: app.packet.goprosetrequest-command %d"%(command))
            logger.log("[gopro-test]: app.packet.goproextendedrequest-value1 %d"%(value1))
            logger.log("[gopro-test]: app.packet.goproextendedrequest-value2 %d"%(value2))
            logger.log("[gopro-test]: app.packet.goproextendedrequest-value3 %d"%(value3))
            logger.log("[gopro-test]: app.packet.goproextendedrequest-value4 %d"%(value4))
            self.sendGoProCommand(command, (value1, value2, value3, value4))

    # Send a photo event with current time and location.
    def sendPhotoEvent(self): 
        now = monotonic.monotonic()
        logger.log("[gopro]: send photo event.  now is %d"%now)

        if self.shotMgr.vehicle.location.global_relative_frame is not None:
            pkt = struct.pack("<IIddfI", app_packet.GOPRO_PHOTO, 24, 
                self.shotMgr.vehicle.location.global_relative_frame.lat, 
                self.shotMgr.vehicle.location.global_relative_frame.lon, 
                self.shotMgr.vehicle.location.global_relative_frame.alt, 
                monotonic.monotonic())

            self.addPhotoLog(
                self.shotMgr.vehicle.location.global_relative_frame.lat, 
                self.shotMgr.vehicle.location.global_relative_frame.lon, 
                self.shotMgr.vehicle.location.global_relative_frame.alt, 
                monotonic.monotonic())
        else:
            pkt = struct.pack("<IIddfI", app_packet.GOPRO_PHOTO, 24, 0, 0, 0, monotonic.monotonic())

        self.shotMgr.appMgr.sendPacket(pkt)

    # Add an entry to /log/photo.log
    def addPhotoLog(self, lat, lon, alt, time):
        f = open("/log/photo.log", "a")
        f.write("%.6f,%.6f,%.3f,%d\n" % (lat, lon, alt, time))
        f.close();


    # packages up our entire current state and sends it to the app
    def sendState(self):
        logger.log("[gopro]: sending Gopro state to app")

        # Because of a bug in version 1.2 and below of the iOS app, the
        # version 1 packet must be filled with zeros to avoid a memory
        # corruption crash.
        # 2 unsigned shorts for a header, 26 unsigned bytes, then 5 unsigned shorts
        pkt = struct.pack('<IIBBBBBBBBBBBBBBBBBBBBBBBBBBHHHHH', app_packet.GOPRO_V1_STATE, 36, \
            GOPRO_V1_SPEC_VERSION,
            self.model,
            self.status,
            self.isRecording,
            self.captureMode,
            # for now, all the rest is yet to be defined
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0
            )
        self.shotMgr.appMgr.sendPacket(pkt)

        # Now also send a version 2 packet to include the additional GoPro settings
        # 2 unsigned shorts for a header, 26 unsigned bytes, then 5 unsigned shorts
        pkt = struct.pack('<IIBBBBBBBBBBBBBBBBBBBBBBBBBBHHHHH', app_packet.GOPRO_V2_STATE, 36, \
            GOPRO_V2_SPEC_VERSION,
            self.model,
            self.status,
            self.isRecording,
            self.captureMode,
            self.videoFormat,
            self.videoResolution,
            self.videoFrameRate,
            self.videoFieldOfView,
            self.videoLowLight,
            self.photoResolution,
            self.photoBurstRate,
            self.videoProtune,
            self.videoProtuneWhiteBalance,
            self.videoProtuneColor,
            self.videoProtuneGain,
            self.videoProtuneSharpness,
            self.videoProtuneExposure,
            self.enabled,
            # for now, all the rest is yet to be defined
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0
            )
        self.shotMgr.appMgr.sendPacket(pkt)

    # Tell the gimbal to turn Gopro comms on/off depending on if our internal
    # flag is set to on/off
    def setGimbalEnabledParam(self):
        value = 1.0 if self.enabled else 0.0

        logger.log("[gopro]: sending gimbal enabled param (GMB_GP_CTRL) to %f"%(value))

        msg = self.shotMgr.vehicle.message_factory.param_set_encode(
            0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,    # target system, target component
            "GMB_GP_CTRL", value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        self.shotMgr.vehicle.send_mavlink(msg)

    # When the app tells us to enable/disable Gopro controls, we do:
    # 1.  set our internal flag
    # 2.  Tell settings to write out this value
    # 3.  Set the gimbal's parameter
    def setGoProEnabled(self, enabled):
        self.enabled = enabled
        value = 1 if enabled else 0
        settings.writeSetting("GoProEnabled", str(value))
        self.setGimbalEnabledParam()
        logger.log("[gopro]: We have set GoProEnabled to %d"%(value))
        
    # Send the Gopro command via wifi to the Arduino
    def setGoProCommandArduino(self, commandnum):
        bitarr = list(reversed(list(bin(commandnum)[2:].zfill(7))))
        #logger.log("[gopro-arduino]: bitarray received before sending to Arduino: %s"%(bitarr))
        try:
            # send the bitstring to the Arduino pins
            self.arduinoBoard.digital_write(GOPRO_FLUSH, 0)
            for i in range(7):
                self.arduinoBoard.digital_write(i, int(bitarr[i]))
                # time.sleep(0.1)
            #flush and execute the command, then reset 
            self.arduinoBoard.digital_write(GOPRO_FLUSH, 1)
            time.sleep(0.3)   #give time to execute on the Gopro side
            self.arduinoBoard.digital_write(GOPRO_FLUSH, 0)
            logger.log("[gopro-arduino]: bitarray sent to Arduino: %s"%(bitarr))
        except:   
            logger.log("[gopro-arduino]: Error in communication to Arduino")
            
    def goproModelType(self, data): 
        #logger.log("[gopro-arduino]: handshake status received from arduino: %s"%(data[2]))
        # if (data[2] == True): #if handshake is True - meaning a new value has been determined
        x = str(self.arduinoBoard.digital_read(OUTPUT1)) + str(self.arduinoBoard.digital_read(OUTPUT2)) 
        getmodel = int(x, 2)
        logger.log("[gopro-arduino]: model received from arduino: %s"%(getmodel))
        if getmodel == 0:
            self.status = mavutil.mavlink.GOPRO_HEARTBEAT_STATUS_DISCONNECTED
            self.model = MODEL_NONE #no valid gopro detected
            exceptStr = "no supported camera"
        elif getmodel == 1:
            self.status = 2
            self.model = MODEL_HERO4_BLACK #as long as MODEL_HERO5_BLACK is not supported in Solex
            exceptStr = "Gopro Hero 5 black or Session connected"
        elif getmodel == 2:
            self.status = 2
            self.model = MODEL_HERO4_BLACK #MODEL_HERO6_BLACK
            exceptStr = "Gopro Hero 6 black connected"
        elif getmodel == 3:
            self.status = 2
            self.model = MODEL_HERO4_BLACK #MODEL_HERO7_BLACK
            exceptStr = "Gopro Hero 7 black connected"
        else:
            # in the meantime we treat all others as a Hero 4
            self.model = MODEL_HERO4_BLACK    # send status to app
            exceptStr = "no valid camera assuming Hero 4"
        # update Gopro model in Solex
        logger.log("[gopro]: rstatus %d."%(self.status))
        logger.log("[gopro]: rmodel %d."%(self.model))
        self.sendState()
        if self.shotMgr.appMgr.isAppConnected():
            packet = struct.pack('<II%ds' % (len(exceptStr)), app_packet.SOLO_MESSAGE_SHOTMANAGER_ERROR, len(exceptStr), exceptStr)
            self.shotMgr.appMgr.client.send(packet)
            # sleep to make sure the packet goes out
            time.sleep(1)   