#!/bin/sh
SUCCESS=0
export LD_LIBRARY_PATH=./../eSPDI:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./../eSPDI/Debug:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./../eSPDI/turbojpeg/TX2/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./../eSPDI/opencv/TX2/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./../eSPDI/usb-QCOM_64/lib/:$LD_LIBRARY_PATH

cd out_img
if [ "$?" -ne $SUCCESS ]
then
    echo "creat out img folder"
    mkdir out_img
else
    cd ../
    rm -rf ./out_img/*.*
	  echo "run ARM test..."
fi
./test_arm64_sunplus
