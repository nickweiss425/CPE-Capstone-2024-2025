NGCP Flight Control GUI Installation instructions:\
\
Starting from Windows Command Line:\
docker run --name gui -it osrf/ros:humble-desktop\
git clone https://github.com/nickweiss425/CPE-Capstone-2024-2025.git\
\
Container:\
sudo apt update\
sudo apt upgrade\
sudo apt install qtquickcontrols2-5-dev\
sudo apt install qtlocation5-dev\
sudo apt install qtpositioning5-dev\
sudo apt install qtmultimedia5-dev\
sudo apt install x11-xserver-utils\
sudo apt install qml-module-qtquick-window2\
\
Windows:\
choco install vcxsrv (elevated command prompt)\
Run xlaunch from start menu\
docker start -i gui\
Get Virtual ethernet adapter with ipconfig (should be under something similar to "Ethernet adapter vEthernet (Default Switch):")\
\
Container:\
Put "export DISPLAY=IP_FROM_LAST_STEP:0.0" into ~/.bashrc\
sudo apt install qml-module-qtquick2\
sudo apt install qml-module-qtlocation\
sudo apt install qml-module-qtpositioning\
sudo apt install qml-module-qtquick-controls2\
git checkout qt-gui\
cd into the repo\
cd into gui\gui_messages\
colcon build
source install/setup.sh
cd ..
colcon build when inside CPE-Capstone-2024-2025/gui
source install/setup.sh
cd build/flight_control/
./flight_control
