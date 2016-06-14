# Rules and Tips (Draft) #

### Code Style ###
http://wiki.ros.org/StyleGuide is recommended.

### Standard Frame Definition ###
         IN UAV VIEW  
     
          z(up)   x (forward)
            ^    /
            |   /
            |  /
            | /
    y <-----|/ 
    (left)

### Basic Rules ###
* 1-based index is used for "*id" variables for the reason of comparision with "0" when using unsigned type.
* Customized message types are not encouraged for the reason of visualization and code distrubution, so please try to use a ros-standard message first. If it is essential to create a new message type for system integration, please add it to [quadrotor_msgs](https://github.com/jchenbr/quadrotor_msgs).

### Message Header "frame_id" Rules ###
* global frame: "world" or "map"
* body frame: "body"
* sensor frame: "laser" "sonar" "image" etc.

### Odometry Message "nav_msgs/Odometry" Rules ###
* Use the standard frame.

### Customized Odometry Message Rules ###
* Use [quadrotor_msgs](https://github.com/jchenbr/quadrotor_msgs) as sources.
* Keyframe-based estimators are expected to use this customized odometry message. Other nodes can use the ros standard "nav_msgs/Odometry".
* Field "curodom" for current odometry.
* Field "kfodom" for keyframe odometry related to the current frame.
* Field "kfid" for keyframe id. *1-based*.
* Field "status" for the status of the current odometry. Values are defined as constants in the message.

### Customized PositionCommand Message Rules ###
* Field "trajectory_id" for trajectory id. *1-based*.
* Field "trajectory_flag" is used to identify the status of the command. Values are defined as constants in the message.

### General Tips ###
* ROS is a very powerful tool with excellent documents and an active community. Read the wiki pages. Google your problems.
* Use a version control tool to manage the code, except that you can remember the difference between today's code and yesterday's code, and code in NUC and code in your USB disk. Some free online services: [Github](https://github.com/) [Bitbucket](https://bitbucket.org/)
* [terminator](http://gnometerminator.blogspot.com/p/introduction.html) / alt+[12345] in gnome-terminal for host PC. [tmux](https://tmux.github.io/) with [tmux-resurrect](https://github.com/tmux-plugins/tmux-resurrect) for onboard device.
* bash: alias cmk="cd ~/catkin_ws; catkin_make; cd -" to avoid changing directory before catkin_make.

### Interface Tips ###

* Never hard code a path in the code, such as ```load_calib_param("/home/foo/25000681.yaml")```. Instead, please use a rosparam to load the file path. You can either use \<param .../\> or \<rosparam>...\</rosparam>
* Use private topics in the code and remap them to global ones in the launch file. Google "ros namespace" if your are not familiar with it.

### Visualization and Printing Tips ###

* Be careful when using ```cv::imshow(...)``` because it will raise an exception when there is no display (e.g. NUC without a screen). Provide a convenient switch (rosparam, preprocessor directive) to turn it on or off.
* Many useful tools in rosconsole. See [rosconsole wiki page](http://wiki.ros.org/rosconsole)

### Tips for Debugging ROS Applications (Oops! Segmentfault!)" ###

#### Debug compile problem ####
* Add ```set (CMAKE_VERBOSE_MAKEFILE true)``` to see verbose output of compile

#### Use gdb ####

* Add `-g` in your CMakefiles: ```set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall```
* Add `launch-prefix="gdb --args"` and make sure that `output="screen"` in your `<node>` tag attributes. `output="log"` will redirect output of gdb to a log file, and you will not see it.
* Some useful gdb commands: r, c, b, bt, frame, q. Google "gdb" for detailed information.

#### Use backward-cpp ####
* See https://github.com/bombela/backward-cpp for details.
* Install bfd
* Include backward.hpp in your code and set the #define flag
* Copy contents in backward.cpp in your code OR compile with it
* Add bfd in your ```target_link_libraries```

### Steps for Multiple Machines Communication on ROS ###

Before continuing, please google "ROS multiple machines" and read the tutorial/wiki to *get general concepts*. But the toturial is confusing, so come back when you do it step by step.

#### Edit `/etc/hosts/` with static ip address ####

1. Edit the ```/etc/hosts``` on **BOTH** local and remote machines. Add hostnames and their ip addresses in the host files. After editing, you can test it with ping. Make sure that they can ping each other by using **HOSTNAME** (NOT ip address).

2. For the machine which is expected to connect to a remote roscore, set the environment variable: ``` export ROS_MASTER_URI=http://the-remote-machine-hostname:11311```

3. You can add **the export command** or **an alias** in the ```~/.bashrc``` for convenience.

#### Using [mDNS](https://help.ubuntu.com/community/HowToZeroconf) feature of Ubuntu for dynamic ip ####

0. Since ubuntu is equipped with [mDNS](https://help.ubuntu.com/community/HowToZeroconf), if your local and remote machine are in the same subnet, they should be able to resolve each other with HOSTNAME.local. For example, your onboard device has hostname `uav` with ip address `192.168.1.123`, and your laptop has hostname `laptop` with ip address `192.167.1.124` in the same local network. Try to ```ping uav.local``` on your laptop and ```ping laptop.local``` on your onboard device. They should work well.

1. Use ```ping OTHER-DEVICE-HOSTNAME.local``` to verify the connection.

2. On your onboard device, ```export ROS_HOSTNAME=YOUR-ONBOARD-DEVICE-HOSTNAME.local```, substitute YOU-ONBOARD-DEVICE-HOSTNAME with the real hostname.

3. On your laptop, ```export ROS_HOSTNAME=YOUR-LAPTOP-HOSTNAME.local```, substitute YOUR-LAPTOP-HOSTNAME with the real hostname.

4. For the machine which is expected to connect to a remote roscore, set the environment variable: ``` export ROS_MASTER_URI=http://THE-REMOTE-MACHINE-HOSTNAME.local:11311```

5. Some tips: steps 3,4 can be added to your *~/.bashrc* with no side effects. You can use an alias for step 4. 

I've also wrote a plugin for [powerline-shell](https://github.com/milkbikis/powerline-shell). My plugin will show up the **hostname in ROS_MASTER_URI** on the powerline-shell as a reminder. See [groundmelon/powerline-shell](https://github.com/groundmelon/powerline-shell) if you'd like to use and improve it.

### IP Address Solution with Direct Wired Connection between Onboard Device and Laptop ###

In some situations, there is no router which can help you find out what the ip address is. Two ways to find out the onboard device's ip address:

1. Use screen,mouse,keyboard to set ```eth0/eth1/eth*``` with a static ip. Or use ssh to edit /etc/NetworkManager/system-connections/xxx to give ```eth0/eth1/eth*``` a static ip. Give your laptop's ```eth0/eth1/eth*``` another static ip within the same subnet.

2. Configure your laptop such that it can assign an ip address to the opposite device. Run a dhcp server on your ```eth0/eth1/eth*```. See https://help.ubuntu.com/lts/serverguide/dhcp.html for details.
