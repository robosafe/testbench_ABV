#!/bin/bash

# The script runs a set of tests in which the human has a 1% chance of disengaging from the task.
#
# To run this script, open a terminal, navigate to the bert2_gazebo folder
# (created when you cloned this git), and type ". shell_scripts/handover.sh".
#
# Code coverage results will be summarised in your/path/bert2_gazebo/stats.txt
# and detailed in various files in your/path/bert2_gazebo/covhtml/.
# Assertion monitor results will be saved in text files in your/path/bert2_gazebo/.

ROBOT_PYSCRIPT="robot"
SUBJ="TEST"


(roscore & echo $! >> /tmp/mainpids) &
(sleep 5; roslaunch bert2_gazebo bert2_gazebo.launch gui:=false headless:=true >> /tmp/simulator_outr & echo $! >> /tmp/mainpids) &
#(sleep 5; roslaunch bert2_gazebo bert2_gazebo.launch >> /tmp/simulator_outr & echo $! >> /tmp/mainpids) &

COUNTER=0

while [ $COUNTER -lt 100 ]; do

  if [ ! -z "$COUNTER" ];
    then ((COUNTER++));
    else COUNTER=1;
  fi

  rm -f /tmp/rospids

  echo "** test_counter="$COUNTER

  # Remove old coverage file so that we only check fresh ones: 
  rm -f $PWD$"/cov/covhtml/scripts_"$ROBOT_PYSCRIPT$"_py.html"
    
  (rosrun bert2_gazebo monitor2.py $COUNTER>> /tmp/monitor2outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor3.py $COUNTER>> /tmp/monitor3outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor4.py $COUNTER>> /tmp/monitor4outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor5.py $COUNTER>> /tmp/monitor5outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor6.py $COUNTER>> /tmp/monitor6outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_speed.py $COUNTER>> /tmp/monitor_speed_outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_speed_after_reset.py $COUNTER>> /tmp/monitor_speed_after_reset_outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_speed_near_human.py $COUNTER>> /tmp/monitor_speed_near_human_outr$COUNTER & echo $! >> /tmp/rospids) &

  (rosrun bert2_gazebo monitor_no_accidental_drop_b.py $COUNTER>> /tmp/monitor_no_accidental_drop_outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_handover_success_b.py $COUNTER>> /tmp/monitor_handover_success_outr$COUNTER & echo $! >> /tmp/rospids) &

  (rosrun bert2_gazebo monitor_gaze_sensed_as_correct_b.py $COUNTER>> /tmp/monitor_gaze_sensed_as_correct_outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_pressure_sensed_as_correct_b.py $COUNTER>> /tmp/monitor_gaze_sensed_as_correct_outr$COUNTER & echo $! >> /tmp/rospids) &
  (rosrun bert2_gazebo monitor_location_sensed_as_correct_b.py $COUNTER>> /tmp/monitor_gaze_sensed_as_correct_outr$COUNTER & echo $! >> /tmp/rospids) &
  if [ $COUNTER == 100 ];
    then (rosrun bert2_gazebo human_2.py stim_disengage >> /tmp/human1outr$COUNTER) &
    else (rosrun bert2_gazebo human_2.py stim_smoothRun >> /tmp/human2outr$COUNTER) &
  fi

  (rosrun bert2_gazebo object.py & echo $! >> /tmp/rospids) &

  # Gazebo-based sensing:
  (rosrun bert2_gazebo sensors_2.py >> /tmp/sensors_2outr$COUNTER & echo $! >> /tmp/rospids) &

  (rosrun bert2_gazebo $ROBOT_PYSCRIPT$".py" >> /tmp/robotoutr$COUNTER)
  #(rosrun bert2_gazebo $ROBOT_PYSCRIPT$".py")

  echo "The robot has finished. Sleeping to allow monitors to check late events."
  sleep 10
  echo "Reawoken.  Checking coverage."
  date "+%F %T" >> cov/stats.txt
  echo "** COUNTER="$COUNTER", SUBJ="$SUBJ >> cov/stats.txt
  python $PWD$"/scripts/check_code_coverage.py" $ROBOT_PYSCRIPT
  echo "Coverage checking complete."

  cat /tmp/rospids | xargs kill

done

cat /tmp/mainpids | xargs kill
