#!/bin/bash

if [ "$PWD" != "$HOME/catkin_ws" ]; then
    read -p "Current directory ($PWD) is not the usual catkin_ws location ($HOME/catkin_ws). Continue? [yn]: " yn
    case $yn in
        [Yy]* ) ;;
        * ) exit;;
    esac
fi

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw2.zip"
echo "Creating ${sunetid}_hw2.zip"
zip -q "${sunetid}_hw2.zip" "submit_hw2_ros.sh"
zip -qd "${sunetid}_hw2.zip" "submit_hw2_ros.sh" # making an empty zip file

for fname in "src/turtlebot_control/scripts/supervisor.py" \
             "src/turtlebot_control/scripts/controller.py"
do
    if [ -f $fname ]; then
        zip "${sunetid}_hw2.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

if [ -f "hw2_4.bag" ]; then
    echo "Found hw2_4 bag ./hw2_4.bag"
    zip "${sunetid}_hw2.zip" "hw2_4.bag"
else
    found=false
    for bag in $(find -name \*.bag)
    do
        topics=$(rostopic list -b $bag)
        if [[ $topics == *"position_goal"* ]] && [[ $topics == *"tf"* ]]; then
            echo "Found hw2_4 bag $bag; copying to ./hw2_4.bag"
            cp $bag "hw2_4.bag"
            zip "${sunetid}_hw2.zip" "hw2_4.bag"
            found=true
            break
        fi
    done
    if [ "$found" = false ]; then
        read -p "hw2_4 bag not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW2 2>/dev/null | grep -m 1 ${sunetid}_ros_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "${sunetid}_hw2.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW2/${sunetid}_ros_submission_$subnum.zip" 2>/dev/null
