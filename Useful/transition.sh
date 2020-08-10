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

echo "Input width and height: e.g: 1440,1080"

read size

if [ ${#size} -lt 4 ]; then
    width=1440
    height=1080
    echo "Output video size set as (1440, 1080)."
else
    array=(${size//,/ })
    width=${array[0]}#
    height=${array[1]}
    echo "Output video size set as ($width, $height)"
fi

sed -i "27c add_executable(Task1 src/transit.cc)" CMakeLists.txt

cd build
cmake .. && make -j8
echo "Input file[$path$extension] >> output file[$opath$extension]. Processing..."
cd ..
./Task1 $path$extension $opath$extension $width $height

echo "Process completed."
exit