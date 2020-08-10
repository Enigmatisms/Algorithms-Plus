#!/bin/bash
# 消除时钟偏差(转移可能造成时钟偏差)

function deSkew(){
    now_time=`date "+%s"`
    mod_time=`date -r $1 "+%s"`
    if (( now_time > mod_time )); then
        echo "File \'$1\' is modified in the future."
        sed -i '$a\ ' $1    # sed 中 第一行是 '1' 增加为 '1a\xxx' xxx为内容
        sed -i '$d' $1      # 最后一行删去  行末在sed 中为 $ 删去为 'xd', x表示行号
    fi
}

function dirRecursive(){
    # $1 参数1是需要递归查找的路径
    # $2 参数2是需要替换的字符标识
    for file in `ls $1`; do
        if [ -d "$1/$file" ]; then
            dirRecursive $1/$file "$2"
        else
            extension="${file##*.}"     # 取最后一个.的后部分
            exts=("cc" "cpp" "hpp" "h" "yaml" "c")

            # 从数组中删去 extension 如果数组的长度减小，则此文件的后缀在exts中
            if [[ ${exts[@]/${extension}/} != ${exts[@]} ]]; then
                # replace 文件
                deSkew $1/$file "$2"
            fi
        fi
    done
}

echo "De-skewing for this directory..."
dirRecursive $1
echo "Process completed."