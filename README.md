# 3DR-Solo-GoPro-Hero-black-5-6-7-Control
Necessary APIs and functions to control a Hero black 5, 6 or 7 Black from3DR Solo app or Solex

Note: Work in progress, use at your own risk!
This repro will get updated with new features and bug fixes over time.

Based on the work of Konrad Iturbe (KonradIT) Thank you for your great work!
https://github.com/KonradIT/goprowifihack/blob/master/README.md

Video see here: 
- https://youtu.be/H4vh83SqS-Y
- https://youtu.be/H4vh83SqS-Y
- https://youtu.be/dY4mVUrN4QM

# 1. Prerequisites
- Requires an wifi capable Arduino such as the MKR1000
- A Micro USB cable to connect the MKR1000 to the IMX6 of Solo. You can find a micro to micro USB cable on Amazon or eBay  
- A Solo breakoutboard with an USB connector (not mandatory, but will save you a lot of time to break out the USB port yourself)
- A red and a green LEDs to display the wifi status

# 2. How to install

## 2.1. Hardware setup

Solder the red LED to the A1 pin and the green LED to the A3 pin of the MKR1000. Use a 50 Ohm resistor to avoid overload.
Connect the Arduino MKR1000 USB to the IMX USB port.

LED Status indicator:
- Red and green LED off,green on board LED on: board is powered, waiting for Solo to finalize GoproManager initialization.
- Blinking red and green LED: wifi card error
- Solid red and no green LED: Initialized but no Gopro in range
- Solid red and solid green LED: connected to Gopro, trying to wake it up if in sleep mode
- Solid green and no red LED: connection to Gopro established.
- Solid green and flashing red LED: indicates data transfer from Solo to GoPro

## 2.2. Software setup
   
### 2.2.1. Install Pymata on your Solo

You must have PyMata version 2.1 installed on Solo. Newer versions than 2.1 do not properly install on Solo. To download PyMata, go here: https://github.com/MrYsLab/pymata

### 2.2.2. Install the Arduino Sketch on the MKR 1000
Ensure that you have installed the required Arduino MKR1000 libraries in the Arduino workbench. In particular, the wifi101 lib and the ArduinoJSON lib. Both libraries can be selected and downloaded in the Arduino workbench library manager tool.

Load the sketch into the Arduino workbench and change the Wifi settings according to your GoPro in the arduino_secrets.h file. You can enter the ssid and password credentials for up to 3 different Gopros.

Then upload the sketch.

### 2.2.3. Install necessary files on Solo

Note: This mod is compatible with stock firmware and OpenSolo, green cube and the stock black cube.

Important Note: backup all Solo gopromanager and shotmanager files before uploading the new code so that you can revert back in case something goes south.

SSH into Solo, go to the /usr/bin directory and upload the gopromanager.py and goproconstants.py files
Note: by default, gopromanager tries to connect through the Arduino.
If you want to switch back to standard functionality with a Hero4 connected through the 3DR Gimbal, then set the constant:

    SOLO_MOD = "GOPRO"
to

    SOLO_MOD = ""

in the gopromanager.py file.

## 2.3. Setup your Gopro
 
A current limitation is that Solo does not know the Gopro settings when we turn it on. Therefore, gopromanager assumes a default setting of your Gopro and you need to ensure that you have the following parameters at startup set in your Gopro5, 6 or 7 Black:

- Video Resolution 1080p; FOV Linear, 60FPS
- Protune and low light: on
- Photo resolution 12MP linear

You may change those defaults of course in the gopromanager.py file. All other parameters will be used as set by default in your Gopro. 

Don't foreget to turn on your gopro wifi.

> Note: Gopro and Solo are both broadcasting over 2.4 GHz wifi on separate channels. While I have not encountered any interference or issues, it cannot be ruled out that interference could happen under certain circumstances.
To eliminate possible interference with the GPS, use the 3DR V2 shield or even better the dedicated 3M EMI protection fabric to shield the GPS and Solo main board from the Gopro. Do NOT USE the stock copper shield or any cardboard mods. The 3M fabric can be purchased at e.g. mRo. 

# 3. Features

Compatible with Gopro Hero 5 Black and Hero 5 Session, Hero 6 Black and Hero 7 Black.
All feature settings in Solex are supported with the exceptions are listed in the next section.
The software supports in addition the major cool features of the Hero 5, 6, and 7 models, such as video stabilization, linear mode, 4K60FPS, Timewarp, Superphoto, and so on. Note that such features are not (yet) accessible threough Solex since some extensions to the current camera menus are needed.

- The software  recognizes the camera model connected.
- You can hot-swap between all registered Gopro cameras w/o restarting Solo
- Voice output and text prompt on the connected camera model in Solex.
- Gopro will automaticallly turn on when Solo is powered on if the camera is in sleep mode
- Gopro battery monitoring: you will receive a warning message when the Gopro battery is getting low

# 4. Exceptions and known limitations

The newer Gopro models differ in several areas from the Hero 4. In order to get it working with the current Solex version, we set the GoPro model to a Hero 4, which causes that some settings are not accessible in the Solex menus:

- Newer video formats along with new frame rates and FOV settings
- Newer features like Linear, Hyperlapse, EIS, RAW implemented but not yet accessible 
- Whitebalance, ISO, Color, manual exposure settings (also never worked with the original software)
- The Gopro will not turn off automatically. (Current limitation is in the Arducopter code - may be fixed with AC3.7)

# 5. To do list
- Check if menu extensions to support the new features can be added to Solex
- Work on a mod to fit the HERO5/6/7 into the Solo gimbal (someone need to sponsor a broken or trashed gimbal to try out the modding)
