#!/bin/sh
echo "run NVIDIA TX2"
export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/opencv/TX2/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/turbojpeg/TX2/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/self_calibration/Self_Calibration_API/aarch64/lib/:$LD_LIBRARY_PATH
sync
./DMPreview_TX2
