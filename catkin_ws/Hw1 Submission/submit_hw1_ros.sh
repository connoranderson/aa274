#!/bin/bash

if [ "$PWD" != "$HOME/catkin_ws" ]; then
    read -p "Current directory ($PWD) is not the usual catkin_ws location ($HOME/catkin_ws). Continue? [yn]: " yn
    case $yn in
        [Yy]* ) ;;
        * ) exit;;
    esac
fi

read -p "Please enter your SUNetID: " sunetid

rm -f "$sunetid.zip"
echo "Creating $sunetid.zip"
zip -q "$sunetid.zip" "submit_hw1_ros.sh"
zip -qd "$sunetid.zip" "submit_hw1_ros.sh" # making an empty zip file

for fname in "src/turtlebot_control/scripts/publisher.py" \
             "src/turtlebot_control/scripts/subscriber.py" \
             "src/turtlebot_control/scripts/controller.py"
do
    if [ -f $fname ]; then
        zip "$sunetid.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

if [ -f "random_strings.bag" ]; then
    echo "Found random_strings bag ./random_strings.bag"
    zip "$sunetid.zip" "random_strings.bag"
else
    found=false
    for bag in $(find -name \*.bag)
    do
        topics=$(rostopic list -b $bag)
        if [[ $topics == *"random_strings"* ]]; then
            echo "Found random_strings bag $bag; copying to ./random_strings.bag"
            cp $bag "random_strings.bag"
            zip "$sunetid.zip" "random_strings.bag"
            found=true
            break
        fi
    done
    if [ "$found" = false ]; then
        read -p "random_strings bag not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
fi

if [ -f "turtlebot.bag" ]; then
    echo "Found turtlebot bag ./turtlebot.bag"
    zip "$sunetid.zip" "turtlebot.bag"
else
    found=false
    for bag in $(find -name \*.bag)
    do
        topics=$(rostopic list -b $bag)
        if [[ $topics == *"model_states"* ]] && [[ $topics == *"navi"* ]]; then
            echo "Found turtlebot bag $bag; copying to ./turtlebot.bag"
            cp $bag "turtlebot.bag"
            zip "$sunetid.zip" "turtlebot.bag"
            found=true
            break
        fi
    done
    if [ "$found" = false ]; then
        read -p "turtlebot bag not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW1 2>/dev/null | grep -m 1 ${sunetid}_ros_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "$sunetid.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW1/${sunetid}_ros_submission_$subnum.zip" 2>/dev/null



