#!/bin/bash

# This script replays rosbag recordings of user experiments, applying offline assertion monitoring.
# Run it from the bert2_gazebo folder using 
#       . shell_scripts/handover_exp_playback.sh
# Make sure you edit the MYPATH variable below to fit wherever your recordings are. We assume the files
# are stored with the structure implied by line 59 below (subject folder and file naming).
#
# Created by David Western, July 2016.

MYPATH="/home/username/examplePath"

(roscore & echo $! >> /tmp/mainpids) &

sleep 7

rosparam set is_this_a_simulation 0
rosparam set use_sim_time true

SUBJ=0

while [ $SUBJ -lt 10 ]; do


  ((SUBJ++))

  COUNTER=0

  while [ $COUNTER -lt 10 ]; do

    ((COUNTER++))

    rm -f /tmp/rospids

    sleep 10

    ((TRACE=($SUBJ-1)*10+$COUNTER))
    echo "** subject="$SUBJ", test="$COUNTER", trace="$TRACE

    # Remove old coverage file so that we only check fresh ones: 
    rm -f $PWD$"/cov/covhtml/scripts_"$ROBOT_PYSCRIPT$"_py.html"
  
    (rosrun bert2_gazebo monitor3.py $TRACE>> /tmp/monitor3outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor4.py $TRACE>> /tmp/monitor4outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor2.py $TRACE>> /tmp/monitor2outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor5.py $TRACE>> /tmp/monitor5outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor6_exp.py $TRACE>> /tmp/monitor6outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor_speed_exp.py $TRACE>> /tmp/monitor_speed_outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor_speed_after_reset_exp.py $TRACE>> /tmp/monitor_speed_after_reset_outr$TRACE & echo $! >> /tmp/rospids) &
    (rosrun bert2_gazebo monitor_speed_near_human_exp.py $TRACE>> /tmp/monitor_speed_near_human_outr$TRACE & echo $! >> /tmp/rospids) &

    if [ $SUBJ -lt 10 ]; 
        then SUBJCODE="0"$SUBJ;
        else SUBJCODE=$SUBJ;
    fi
    if [ $COUNTER -lt 10 ]; 
        then TESTCODE="0"$COUNTER;
        else TESTCODE=$COUNTER;
    fi
    rosbag play $MYPATH"/Experiments/recordings/sub"$SUBJCODE"/bagfiles/sub"$SUBJCODE"test"$TESTCODE".bag" --clock 

    cat /tmp/rospids | xargs kill

  done
done

cat /tmp/mainpids | xargs kill
