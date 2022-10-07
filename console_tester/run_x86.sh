#!/bin/sh
SUCCESS=0
export LD_LIBRARY_PATH=./../eSPDI:./../eSPDI/Debug:$LD_LIBRARY_PATH

cd out_img
if [ "$?" -ne $SUCCESS ]
then
	echo "creat out img folder"
	mkdir out_img
else
    cd ../
    rm -rf ./out_img/*.*
	echo "run X86 test..."
fi
./test_x86
