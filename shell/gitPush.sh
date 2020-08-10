#!/bin/bash

for files in *
do
    let file_cnt+=1
done

step_cnt=0
pyarg1=""
pyarg2=""
for i in `ls -a`; do
    name=`stat -c %n $i`
    mod=`stat -c %Y $i`
    
    step_cnt=$[ step_cnt + 1 ]
    if [ ! $step_cnt -eq $file_cnt ]; then
        # 不换行输出
        pyarg1=$pyarg1$"$name,"
        pyarg2=$pyarg2$"$mod,"
    else
        pyarg1=$pyarg1$name
        pyarg2=$pyarg2$mod
    fi
done

addList=$(echo `python gitProcess.py $pyarg1 $pyarg2`)
array=(${addList//,/})

for var in ${array[@]}; do
    git add $var
done

git commit -m "$1"

git push origin master

