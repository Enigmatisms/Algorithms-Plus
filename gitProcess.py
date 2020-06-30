#!/usr/bin/env python
#-*-coding:utf-8-*-
# GitHub上传辅助

import json
import sys

if __name__ == "__main__":
    strarg = sys.argv[1]
    lst1 = strarg.split(",")
    numarg = sys.argv[2]
    lst2 = numarg.split(",")
    for i in range(len(lst2)):
        lst2[i] = int(lst2[i])
    old = dict(zip(lst1, lst2))

    try:
        with open("./gitFileList.txt", "r") as reads:
            now = json.load(reads)
    except FileNotFoundError:
        now = dict()
    else:
        pass
    
    res = []

    for var in old:
        if var in now:
            if old[var] > now[var]:
                res.append(var)
        else:
            res.append(var)
    
    for i in range(max(0, len(res) - 1)):
        print(res[i] + ",")
    if res:
        print(res[-1])

    