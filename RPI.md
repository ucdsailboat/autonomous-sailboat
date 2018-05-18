RPI DOCUMENTATION
CONNECTING TO THE RPI WITH WINDOWS REMOTE DESKTOP CONNECTION
If you're setting up the RPI for the first time, you'll need to set it up for remote desktop connection by following this YouTube video and this blog link to fix the default cursor.

Otherwise, to use a laptop's mouse and keyboard, you'll need to connect to the RPi with Window's built-in Remote Desktop Connection (or equivalent on Linux or Mac).

Connect the RPi to the laptop with HDMI
Log into Window's remote desktop connection
COMPUTER: raspberrypi.local
USERNAME: pi
PASSWORD: eme185
DOWNLOADING ARDUINO LIBRARIES ON RPI
Zip folders of the libraries are in the repository.

Download and unzip the folders for the IMU, GPS, and ANEMOMETER. Make sure your .h and .cpp files are not in another folder or else the Arduino IDE won't find the library header.
Move folders to /usr/share/arduino/libraries/ with command terminal and type the following commands:
cd /home/pi/folderName
sudo mv folderName /usr/share/arduino/libraries/
