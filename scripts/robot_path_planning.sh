#!/bin/bash

gnome-terminal --tab -e "bash -c \"rosrun robot_path_planning mod_a_star1.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_a_star2.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_a_star3.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_a_star4.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_go1.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_go2.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_go3.py; exec bash\"" --tab -e "bash -c \"rosrun robot_path_planning mod_go3.py; exec bash\""
$SHELL
