#!/bin/bash
# README.md fast editing

if [ ! -f "README.md" ]; then
	echo "# README.md" &> README.md
fi

echo "Editing README.md(input \"quit\" to quit):"
echo -e ":\c"

push_judge=1

while [ 1=1 ]; do
	read Str	
	# seperated by "@"
	op=${Str%%@*}
	if [ "$op" = "quit" ]; then
		push_judge=0
		break
	elif [ "$op" = "push" ]; then
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

if [ $push_judge -eq 1 ]; then
    git add README.md

    git commit -m "Update README.md"

    git push origin master
else
    echo $(date "+%Y.%m.%d %T")". Modification saved. File not pushed."
fi
exit
