testbench_ABV

Testbench used to demonstrate assurance-based verification of an object-handover scenario with the BERT2 robot.  All components required to run the simulation are included.

----------------CONTENTS------------------------------------------------

This repo includes five ROS packages:

   bert2_gazebo contains the files used to specifically to implement and test the handover scenario.  Its subfolders include:
      scripts: Control code for robot (in simulation and hardware experiments) and human (in simulation). Code coverage collector.
      src: Additional low-level control modules for robot and human
      shell_scripts: Scripts to run batch mode.
      tests: Test generators and examples of stimuli (test templates with high-level human actions) from model-based test generation.
      launch: Launch files for initialising simulations and hardware experiments.
      monitors: Assertion monitors used to check behaviour of the system.
      meshes, robots, urdf, worlds: Physical modelling of robot, human, and environment.  
   bert2_common, bert2_interface, bert2_robot, moveit_robots: Additional packages to support the BERT2 system, not specific to the handover scenario or testbench.  They include the physical model, motion planning configuration, low-level control and messaging.


----------------------LICENSE--------------

The code we developed is covered by GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007




----------------INSTALLING AND RUNNING THE TESTBENCH---------------------

To start using the bert2_gazebo package to simulate BERT2...



- Install Ubuntu (tested on v14.04)




- Download and install ROS Indigo (full) as per instructions at http://wiki.ros.org/indigo/Installation/Ubuntu.

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
        sudo apt-get update
        sudo apt-get install ros-indigo-desktop-full
        sudo rosdep init
        rosdep update
        echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        sudo apt-get install python-rosinstall


- Install Gazebo2

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update
        sudo apt-get install gazebo2




- Install Coverage Python module (and Pip if you do not have it)

        sudo apt-get install python-pip 
        sudo pip install coverage




- Install the BERT2 ROS Software

Install libraries and tools that the BERT2 ROS software relies on by running the following:

        sudo apt-get update
        sudo apt-get install git python-argparse python-wstool python-vcstools python-rosdep ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control ros-indigo-moveit-full ros-indigo-ros-control ros-indigo-ros-controllers





- Create a ROS workspace (AKA catkin workspace) directory with a name of your choosing, e.g. "bert2_ws", by running the following command in a terminal window

        mkdir -p ~/bert2_ws/src



- Copy all contents of the 'ROS_packages' folder in this repository to ~/bert2_ws/src/




- We build the BERT2 ROS packages by running catkin_make.  

        cd ~/bert2_ws
        catkin_make --pkg bert2_core_msgs
        catkin_make     
        echo "source ~/bert2_ws/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc      



- Launch the simulator

        cd ~/bert2_ws/src/bert2_gazebo
        . shell_scripts/handover_sim.sh

The shell script must be executed from the bert2_gazebo directory, as demonstrated above, to work correctly.

Some error messages relating to bert2_gazebo_motor_controller may appear before the simulator has fully launched, while interacting nodes load. Once the models have spawned, after about 20 seconds the first handover test should commence.  A total of 5 tests will be carried out, as a demo.  Assertion monitor reports will appear in the bert2_gazebo folder as 'assertion*.txt'.  Further error messages will appear when the assertion monitor processes are killed at the end of each test.  These do not indicate a problem with the simulation. 

To run the simulation without visualisation, edit bert2_gazebo/shell_scripts/handover_sim.sh to use the provided alternative version of the 'roslaunch' command (with "gui:=false headless:=true").  Comment out the current version of that command.

If you end the simulation early, use the following commands to end all processes started by the shellscript

        cat /tmp/mainpids | xargs kill
        cat /tmp/rospids | xargs kill



Questions, bugs, comments:  david.western@bristol.ac.uk, dejanira.araizaillan@bristol.ac.uk
