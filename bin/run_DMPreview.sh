#!/bin/sh

echo "run DMPreview tools, please select CPU type: "
echo "=================="
echo "1. X86-64"
echo "2. ARM"
echo "3. TI"
echo "4. NVIDIA TX2"
echo "=================="
echo 

read -p "Please select CPU type (enter: 1(x86_64), 2(armhf_32), 3(TI), 4(NVIDIA TX2/NVIDIA Nano)) : " project

case $project in
        [1]* ) 
        echo "run x86_64"
        sh ./run_DMPreview_X86.sh
        break;;

        [2]* ) 
        echo "run ARM"
        sh ./run_DMPreview_ARM.sh
        break;;

        [3]* ) 
        echo "run TI"
        sh ./run_DMPreview_TI.sh
        break;;

        [4]* ) 
        echo "run NVIDIA TX2"
        sh ./run_DMPreview_TX2.sh.sh
        break;;

        * ) 
        echo "Please select CPU type";;
esac
