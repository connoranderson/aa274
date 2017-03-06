#!/bin/bash

if [ "$PWD" != "$HOME/catkin_ws" ]; then
    read -p "Current directory ($PWD) is not the usual catkin_ws location ($HOME/catkin_ws). Continue? [yn]: " yn
    case $yn in
        [Yy]* ) ;;
        * ) exit;;
    esac
fi

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw3.zip"
echo "Creating ${sunetid}_hw3.zip"
zip -q "${sunetid}_hw3.zip" "submit_hw3.sh"
zip -qd "${sunetid}_hw3.zip" "submit_hw3.sh" # making an empty zip file

for fname in "src/asl_turtlebot/scripts/localization.py" \
             "src/asl_turtlebot/scripts/ekf.py" \
             "src/asl_turtlebot/scripts/ExtractLines.py" \
             "src/asl_turtlebot/scripts/maze_sim_parameters.py" \
             "src/turtlebot_control/scripts/supervisor.py" \
             "src/turtlebot_control/scripts/controller.py" \
             "localization.bag" \
             "map_fixing.bag" \
             "gmapping.bag"
do
    if [ -f $fname ]; then
        zip "${sunetid}_hw3.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

if [ -f "$sunetid.pdf" ]; then
    zip "${sunetid}_hw2.zip" "$sunetid.pdf"
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW3 2>/dev/null | grep -m 1 ${sunetid}_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "${sunetid}_hw3.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW3/${sunetid}_submission_$subnum.zip" 2>/dev/null
