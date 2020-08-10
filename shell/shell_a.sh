#!/bin/bash
#shell script implementation

echo "$0 is now starting to be processed."

num=0

#different loops

for i in {1..100}; do
	num=$[ $num + $i ]
done

echo "The result is ${num}"

cnt=1
num=0

while([ 101 -gt $cnt ]); do
	num=$[ $num + $cnt ]
	cnt=$[cnt+1]
done

echo "The result of while is ${num}"


num=0
for i in $(seq 1 1 100); do
	num=$[ $num + $i ]
done

echo "The result of for seq is $num" 

echo "Input a number (greater than 0, int):"

read num


# read from input and error assertion
while([ $num -lt 1 ]); do
	echo "Input failed. Re-input:"
	read num
done

res=0
for i in `seq 1 1 ${num}`; do
	res=$[res+i]	
done

echo "Sum(1, $num) is $res"

# use of array
declare -a array
array=(1 2 3 4 5)

for i in ${array[@]}; do
	echo "Array: $i"
done

for((i=0;i<${#array[@]};i++)); do

echo "For: ${array[$i]}"

done

#(()) differs from []
echo "Please input a number:"
read num

if((num<100)); then
	echo "i is less than 100: $num"
elif([ $num -gt 200 ]); then
	echo "i is greater than 200: $num"
fi
