#!/bin/bash

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw2.zip"
echo "Creating ${sunetid}_hw2.zip"
zip -q "${sunetid}_hw2.zip" "submit_hw2_123.sh"
zip -qd "${sunetid}_hw2.zip" "submit_hw2_123.sh" # making an empty zip file

for fname in "Problem 1/cal_workspace.py" \
             "Problem 1/cam_calibrator.py" \
             "Problem 1/webcam_calibration.yaml" \
             "Problem 2/ExtractLines.py" \
             "Problem 2/PlotFunctions.py" \
             "Problem 3/detect.py" \
             "Problem 3/classify.py" \
             "Problem 3/retrain.py" \
             "Problem 3/utils.py"
do
    if [ -f "$fname" ]; then
        zip "${sunetid}_hw2.zip" "$fname"
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
else
    echo "Cannot find ./$sunetid.pdf; you must submit your HW2 writeup as $sunetid.pdf in this directory."
    exit
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW2 2>/dev/null | grep -m 1 ${sunetid}_123_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "${sunetid}_hw2.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW2/${sunetid}_123_submission_$subnum.zip" 2>/dev/null