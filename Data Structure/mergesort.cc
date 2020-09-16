#include <iostream>
#include <string.h>
// 归并排序实现

template <typename Ty = double>
void printArr(Ty* arr, int len){
    int i = 0;
    for (i = 0; i < len - 1; i++){
        std::cout << arr[i] << ", ";
    }
    std::cout << arr[i] << std::endl;
}

// 递归实现
template<typename Ty = double>
void mergesort(Ty* arr, int start, int end){
    if (end - start > 2){
        int mid = (int)((start + end) / 2);
        mergesort(arr, start, mid);
        mergesort(arr, mid, end);
        Ty* temp = new Ty [end - start];
        memset(temp, 0, sizeof(Ty) * (end - start));
        int cnt = 0;
        int i = start, j = mid;
        for (; i < mid && j < end;cnt++){       // 多重条件写错了，应该是 &&
            if (arr[j] < arr[i]){
                temp[cnt] = arr[j];
                j++;
            }
            else{
                temp[cnt] = arr[i];
                i++;
            }
        }
        while (i < mid){
            temp[cnt] = arr[i];
            cnt++;
            i++;
        }
        while (j < end){
            temp[cnt] = arr[j];
            cnt++;
            j++;
        }
        for (i = start; i < end; i++){
            arr[i] = temp[i - start];
        }
        delete [] temp;
        printArr(&arr[start], end - start);
    }
    else if (end - start == 2){         // start, start + 1, (end不取，与python切片类似)
        if (arr[end - 1] < arr[start]){ // 注意自己的start end 定义
            std::swap(arr[start], arr[end - 1]);
        }
    }
}

int main(){
    double arr[12] = {9.2, 3.1, 4.3, 1.2, 6.6, 7.5, 8.4, 6.4, 9.0, 9.0, 1.0, 1.2};
    mergesort<double>(arr, 0, 12);
    printArr(arr, 12);
    return 0;
}