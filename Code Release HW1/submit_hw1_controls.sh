#!/bin/bash

read -p "Please enter your SUNetID: " sunetid

rm -f "$sunetid.zip"
echo "Creating $sunetid.zip"
zip "$sunetid.zip" *.py
zip "$sunetid.zip" *.npy

if [ -f "$sunetid.pdf" ]; then
	zip "$sunetid.zip" "$sunetid.pdf"
else
	echo "Cannot find ./$sunetid.pdf; you must submit your HW1 writeup as $sunetid.pdf in this directory."
	exit
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW1 2>/dev/null | grep -m 1 ${sunetid}_controls_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "$sunetid.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW1/${sunetid}_controls_submission_$subnum.zip" 2>/dev/null