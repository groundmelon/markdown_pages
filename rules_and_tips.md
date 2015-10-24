# Rules and Tips (Draft) #

### Code Style ###
http://wiki.ros.org/StyleGuide is recommended.

### Standard Frame Definition ###
         IN UAV VIEW  
     
          z(up)   x (forward)
            |   /
            |  /
            | /

    y <-----|/ 
    (left)

### Basic Rules ###
* 1-based index is used for "*id" variables for the reason of comparision with "0", when unsigned type is used.
* Customized message types are not encouraged for the reason of visualization and code distrubution, so please try to use a ros-standard message first. If it is essential to create a new message type, please add it to [quadrotor_msgs](https://github.com/jchenbr/quadrotor_msgs).

### Message Header "frame_id" Rules ###
* global frame: "world" or "map"
* body frame: "body"
* sensor frame: "laser" "sonar" "image" etc.

### Odometry Message "nav_msgs/Odometry" Rules ###
* Use the standard frame.
* "child_frame_id" is used for flag
    * string("V") for valid
    * string("X") for invalid
    * string("L") for loop-closure

### Customized Odometry Message Rules ###
* Use [quadrotor_msgs](https://github.com/jchenbr/quadrotor_msgs) as sources.
* Keyframe-based estimators are expected to use this customized odometry message. Other nodes can use the ros standard "nav_msgs/Odometry".
* Field "curodom" for current odometry.
* Field "kfodom" for keyframe odometry related to the current frame.
* Field "kfid" for keyframe id. 1-based.
* Field "status" for the status of the current odometry. Values are defined as constants in the message.

### Customized PositionCommand Message Rules ###
* Field "trajectory_id" for trajectory id. 1-based.
* Field "trajectory_flag" is used to identify the status of the command. Values are defined as constants in the message.

### General Tips ###
* ROS is a very powerful tool with excellent documents and an active community. Read the wiki pages. Google your problems.
* Use a version control tool to manage the code, except that you can remember the difference between today's code and yesterday's code, and code in NUC and code in your USB disk. Some free online services: [Github](https://github.com/) [Bitbucket](https://bitbucket.org/)
* terminator / alt+[12345] in gnome-terminal
* bash: alias cmk="cd ~/catkin_ws; catkin_make; cd -" to avoid changing directory before make.


### Interface Tips ###

* Never hard code a path in the code, such as **load_calib_param("/home/foo/25000681.yaml")** Instead, please use a rosparam to load the file path.
* Use private topics in the code and remap them to global ones in the launch file. Google "ros namespace" if your are not familiar with it.

### Visualization and Printing Tips ###

* Be careful when using **cv::imshow()** because it will raise an exception when there is no display (e.g. NUC without a screen). Provide a convenient switch (rosparam, preprocessor directive) to turn it on or off.
* Many useful tools in rosconsole. See [rosconsole wiki page](http://wiki.ros.org/rosconsole)

### Tips for Debugging ROS Applications (Oops! Segmentfault!)" ###

* Add **-g** in your CMakefiles: **set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall**
* Add **launch-prefix="gdb --args"** and make sure that **output="screen"** in your `<node`> tag attributes. **output="log"** will redirect output of gdb to a log file, and you will not see it.
* Some useful gdb commands: r, c, b, bt, frame, q. Google "gdb" for detailed information.


