#!/bin/bash
# The first shell script of mine created on 6.24.2020
# DO NOT RUN THESE COMMANDS FOR "rm -rf" IS INCLUDED!
# TO USE THESE COMMANDS SAFELY, REMOVE #

echo -e "Hello world from Linux Shell.\n"
echo "Pre processing... deleting existing files..."

if [ ! -d "test_d/" ]; then
	mkdir test_d
else
	echo "Test_d already exists."
fi

cd test_d/

for ((i = 1; i < 11; i++))
do 
	if [ -d "d${i}" ]; then
#		rm -rf d$i
	fi
done 

echo "start to process the second for loop"


for i in $(seq 1 1 10)
do
	mkdir d$i
	cd d$i/
	for j in {1..5}
	do 
		mkdir dd$j
	done	
	cd ..
done

echo "Directories created."

echo "The dir to be removed(d(input number in range(1, 11))):"

read num
#rm -rf d$num

echo "Dir deleted."
