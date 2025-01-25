sudo apt install libserial-dev\
newgrp dialout\
cd lora\
colcon build\
source install/setup.bash\
ros2 run lora [subscriber] [publisher] --ros-args -p path:=[path]

