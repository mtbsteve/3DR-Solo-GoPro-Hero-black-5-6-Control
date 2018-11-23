# 3DR-Solo-GoPro-Hero-black-5-6-7-Control
Necessary APIs and functions to control a Hero black 5, 6 or 7 Black from3DR Solo app or Solex

Note: Work in progress, use at your own risk!
This repro will get updated with new features and bug fixes over time.

Based on the work of Konrad Iturbe (KonradIT) Thank you for your great work!
https://github.com/KonradIT/goprowifihack/blob/master/README.md

Video see here: https://youtu.be/H4vh83SqS-Y

1. Prerequisites:
- Requires an wifi capable Arduino such as the MKR1000
- A Micro USB cable to connect the MKR1000 to the IMX6 of Solo.
- A Solo breakoutboard with an USB connector (not mandatory, but will save you to break out the USB port yourself)
- Some LEDs to display the wifi status

2. How to install:

2.1. Hardware setup:

Solder the LEDs to the A1 and A3 pins of the MKR1000. Use a 50 Ohm resistor to avoid overload.
Red goes on A1, green goes on A3.
LED Status indicator:
- Solid red and green LED: wifi card error
- Solid red and no green LED: Waiting to connect to Gopro WiFi
- solid green and no red LED: connected
- solid green and flashing red LED: indicates data transfer from Solo to GoPro

Connect the Arduino MKR1000 USB to the IMX USB port
A 20cm micro to micro USB cable can be found on eg Amazon or eBay for a couple of bucks

2.2. Software setup:
   
    2.2.1. Install Pymata on your Solo.
You must have PyMata version 2.1 installed on Solo. Newer versions than 2.1 do not properly install on Solo. To download PyMata, go here: https://github.com/MrYsLab/pymata-aio

    2.2.2. Install the Arduino Sketch on the MKR 1000
Ensure that you have installed the required Arduino MKR1000 libraries in the Arduino workbench
Load the sketch into the Arduino workbench and change the Wifi settings according to your GoPro. You need to set your:
- SSID
- Password
- MAC Address

Then upload the sketch.

    2.2.3. Install necessary files on Solo

Important Note: backup all Solo gopromanager and shotmanager files before uploading the new code so that you can revert back in case something goes south.

SSH into Solo, go to the /usr/bin directory and upload the gopromanager.py and goproconstants.py files
Note: by default, gopromanager tries to connect through the Arduino.
If you want to switch back to standard functionality with a Hero4 connected through the 3DR Gimbal, then set the constant:
SOLO_MOD = "GOPRO" to
SOLO_MOD = ""

in the gopromanager.py file.

Next, edit shotmanager.py and add the 3 lines below for the connection to the Arduino in the start routine:

logger.log("[shotmanager]: try to open Arduino PyMata")

self.arduinoBoard = PyMata("/dev/ttyACM0", verbose=True)

logger.log("[shotmanager]: Arduino PyMata OPENED - OK")

2.3. Setup your Gopro:
 
A current limitation is that Solo does not know the Gopro settings when we turn it on. Therefore, gopromanager assumes a default setting of your Gopro and you need to ensure that you have the following parameters set in your Gopro:

Capture Mode: Video
Video Format: PAL
Video Resolution: 1080p
Video Frame rate: 50FPS
FOV: Linear (yes!)
LowLifgt: on
Photo Resolution: 12MP wide (limitation with the newer Gopros and Solex at the moment)
Burst Rate 5/1
Protune: on
EV setting 0

You may change those defaults of course in the gopromanager.py file

Don't foreget to turn on your gopro wifi.

Note: Gopro and Solo are both broadcasting over 2.4 GHz wifi on separate channels. While I have not encountered any interference or issues, it cannot be ruled out that interference could happen under certain circumstances. 

3. Features:

All feature settings in Solex are supported with the exceptions are listed in the next section.

4. Exceptions and known limitations:

The newer Gopro models differ in several areas from the Hero 4. In order to get it working with the current Solex version, we set the GoPro model to a Hero 4, which causes that some settings do not work yet:
- Photo resolutions other than 12MP wide listed in the Solex menue. Reason is that the Hero 4 modes (5MP, 7MP) do not exist anymore, insted, the newer models support Linear modes and more.
Note that your Gopro may crash and requires a reboot if you try the Hero 4 photo and burst settings!
- Burst mode other than 12MP wide settings (see above)
- Newer video formats along with new frame rates and FOV settings
- Newer features like Linear, Hyperlapse, EIS, RAW, WDR
- Whitebalance, ISO, Color, manual exposure settings
- Setting of EV and protune in other modes than video (also never worked with a Hero 4)
- The Gopro will not turn on or off automatically.

Note: the code includes a workaround to switch to 1080p Linear by selecting the "narrow" FOV in Solex instead

5. To Dos:

on my to do list:
- Add automatic Gopro model detection.
- Determine a fix for the photo resolution settings and burst mode settings
- Work on an adaptor to fit the HERO5/6/7 into the Solo gimbal

