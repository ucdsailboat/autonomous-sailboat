# HARDWARE DOCUMENTATION 
### CONNECTING TO THE RPI WITH WINDOWS REMOTE DESKTOP CONNECTION 
If you're setting up the RPI for the first time, you'll need to set it up for remote desktop connection by following [this YouTube video](https://www.youtube.com/watch?v=IDqQIDL3LKg) and [this blog link](https://raspberrypi.stackexchange.com/questions/56413/error-problem-connecting-to-raspberry-pi-3-with-xrdp) to fix the default cursor.

To use a laptop's mouse and keyboard, you'll need to connect to the RPi with Window's built in Remote Desktop Connection. 
1. Connect the RPi to the laptop with HDMI
2. Log into Window's remote desktop connection 
    * COMPUTER: raspberrypi.local
    * USERNAME: pi
    * PASSWORD: Eme185

### DOWNLOADING ARDUINO LIBRARIES ON RPI
Zip folders of the libraries are in the repository . 
1. Download and unzip the folders for the IMU, GPS, and ANEMOMETER
2. Move folders to _/usr/share/arduino/libraries/_ with command terminal and type the following commands 
   * cd /home/pi/Downloads
   * sudo mv _folderName_ /usr/share/arduino/libraries/
   
