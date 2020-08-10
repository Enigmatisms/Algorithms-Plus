#!/bin/bash
# 文本替换: 用于工控机文件迁移时需要进行的文件路径更改操作


# / 通常为 sed 的分割符, sed -i 's/old/new/g' filePath
# 但如果替换的是目录（其中含有 /） 则可以换分隔符为 ?
# -i 表示替换而不输出结果
# 其中有一个很坑的地方 shell 的天坑属性 : 在使用变量进行字符串替换时需要加'$var'
# 单引号串内引用变量要使用 '' : '$var' 才能解析变量

function replacePath(){
    dst=`cd;pwd`
    if [ $dst = "/home/sentinel" ]; then
        dst=$dst"/ROSWorkspace"
    fi
    case $2 in
    "sentinel")     # 将sentinel替换为本机(hqy 代码转移到别的设备)
        str1="/home/sentinel/ROSWorkspace"
        str2="/home/sentinel"
        sed -i 's?'$str1'?'$dst'?g' $1
        sed -i 's?'$str2'?'$dst'?g' $1;;
    "cl")           # 将cl替换为本机    (cl 代码转移到别的设备)
        src="/home/cl"
        sed -i 's?'$src'?'$dst'?g' $1;;
    "bili")         # 将bili替换为本机  (gqr 代码转移到别的设备)
        src="/home/bili"
        sed -i 's?'$src'?'$dst'?g' $1;;
    "xjturm")       # 将xjturm替换为本机    (工控机代码转移到其他设备)
        src="/home/xjturm"
        sed -i 's?'$src'?'$dst'?g' $1;;
    esac
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
                replacePath $1/$file "$2"
                echo "File: $1/$file processed"
            fi
        fi
    done
}

dst=`cd;pwd`
echo "Replacing \"$2\" into \"$dst\". Processing..."
dirRecursive $1 "$2"
echo "Process completed."