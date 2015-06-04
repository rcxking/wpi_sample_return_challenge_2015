#!/bin/bash
echo "Parsing calibration for $1"

TARBALL="calibrationdata_$1.tar.gz"
TMP_FILE="/tmp/calibrationdata.tar.gz"
CALIBRATION_FILE=cal_$1.yaml

if [ -e  "$TARBALL" ]
then
  echo "$TARBALL already in directory"
elif [ -e "$TMP_FILE" ]
then
  echo "Copying temporary file $TMP_FILE to $TARBALL"
else
  echo "Could not find $TMP_FILE or $TARBALL"
  echo "Please run calibration to create $TMP_FILE"
  exit 1
fi

tar -xvf $TARBALL ost.txt
mv ost.txt ost.ini
rosrun camera_calibration_parsers convert ost.ini $CALIBRATION_FILE
rm ost.ini
echo "Please edit name in $CALIBRATION_FILE to match camera name"
