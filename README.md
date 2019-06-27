# xbee_bridge

After seeing a mess on rosserial xbee wiki, I decided to reinvent the wheel.

##Usage:

On robot and base station: copy udev/97-xbee.rules to /etc/udev/rules.d to fix xbee path to /dev/xbee_blue and /dev/xbee_green

On base station:
```
roslaunch xbee_base xbee_base.launch [port:=/dev/ttyUSBx] #default to /dev/xbee_blue
```
On robot:
```
roslaunch xbee_robot xbee_robot.launch [port:=/dev/ttyUSBx] #default to /dev/xbee_green
```
