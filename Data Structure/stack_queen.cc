#include <iostream>
#include <stack>
#include <vector>
#include <string.h>
// STL 经典8皇后问题
// 回溯法 stack实现
// 实现的时候思路还是比较乱

struct QueenPos{
    int r;
    int c;
    QueenPos(){;}
    QueenPos(int _r, int _c):
        r(_r), c(_c)
    {
        ;
    }
};

// 检查对角线是否被占用
bool checkDiagonal(int** arr, int r, int c){
    for (int i = 0; r - i >= 0 && c - i >= 0; i++){
        if (arr[r - i][c - i] == 1){
            return false;
        } 
    }
    for (int i = 0; r - i >= 0 && c + i <= 7; i++){
        if (arr[r - i][c + i] == 1){
            return false;
        }
    }
    return true;
}

// 检查行数更小的位置是否存在占用
bool checkUpperRow(int** arr, int r, int c){
    for (int i = 0; i < r; i++){
        if (arr[i][c] == 1){
            return false;
        }
    }
    return true;
}

void drawResult(int** const arr){
    for (int i = 0; i < 8; i++){
        for (int j = 0; j < 8; j++){
            std::cout << arr[i][j] << ", ";
        }
        std::cout << std::endl; 
    }
}

// 八皇后主程序
void Queen(int** arr){
    std::stack<QueenPos> queens;
    int qn = -1;
    while (qn < 7){             // 每一次判定下一行，所以设置qn 最大为7
        bool pop_judge = true;
        for (int c = 0; c < 8; c++){
            if (checkDiagonal(arr, qn + 1, c) && checkUpperRow(arr, qn + 1, c)){
                queens.emplace(qn + 1, c);
                pop_judge = false;
            }
        }
        if (pop_judge){         // 一轮遍历下来，下一行没有可以放置的位置，说明此位置存在问题，需要清除
            do{                 // 清除回溯。如果pop导致本次处理的行与栈顶不一致，说明仍然需要继续pop (上一个位置也不对)
                qn = queens.top().r;
                arr[qn][queens.top().c] = 0;    // 置零
                queens.pop();
            }
            while(queens.size() && qn != queens.top().r);
            if (queens.size() == 0){            // 在解出问题前不可能栈空
                printf("Can not compute final result. Exiting...\n");
                return;
            }
        }
        qn = queens.top().r;    // 更新当前存在皇后的最大行数
        arr[qn][queens.top().c] = 1;
    }
    printf("Final result is computed.\n");
    drawResult(arr);
}

int main(){
    int ** board = new int* [8];
    for (int i = 0; i < 8; i++){
        board[i] = new int [8];
        memset(board[i], 0, 8 * sizeof(int));
    }
    printf("Start to compute 8-queen backtracing problem...\n");
    Queen(board);
    for (int i = 0; i < 8; i++){
        delete [] board[i];
    }
    delete [] board;
    return 0;
}