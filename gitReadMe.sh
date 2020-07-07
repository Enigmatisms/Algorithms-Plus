#!/bin/bash
# README.md fast editing

if [ ! -f "README.md" ]; then
	echo "# README.md" &> README.md
fi

echo "Editing README.md(input \"quit\" to quit):"
echo -e ":\c"


while [ 1=1 ]; do
	read Str	
	# seperated by "@"
	op=${Str%%@*}
	if [ "$op" = "quit" ]; then
		break
	fi
	cont=${Str##*@}
	if [ "$op" = "date" ] || [ "$op" = "DATE" ]; then
		str="> * "	
		out="$str"$(date "+%Y.%m.%d")" $cont"
		echo "$out" >> README.md
	else
		first=${op:0:1}
		str=""
		for((i=0;i<${#op};i++)); do
			str="$str""$first"
		done
		str="$str"" "
		if [ $first = ">" ]; then
			str=$str"* "
		elif [ $first = "-" ]; then
			str="$str""[ ] "
		fi
		str="$str""$cont"
		echo "$str" >> README.md
	fi
	echo -e ":\c"
done

echo "Modification saved."$(date "+%Y.%m.%d %T")
exit
