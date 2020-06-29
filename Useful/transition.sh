#!/bin/bash
#快速视频转码脚本

extension=".avi"
path="cv_output.avi"

echo "Input the name of the video to be processed:"

read path

while [ ! -f $path$extension -o ! "${path%%*.}" == $path ]; do
    echo "No file or directory named: ${path}$extension."
    echo "Please re-input:"  
    read path  
done

echo "Specify the output file name(.avi extension):"

read opath


sed -i "27c add_executable(Task1 src/transit.cc)" CMakeLists.txt

cd build
cmake .. && make -j8
echo "Input file[$path$extension] >> output file[$opath$extension]. Processing..."
cd ..
./Task1 $path$extension $opath$extension

echo "Process completed."
exit
    
