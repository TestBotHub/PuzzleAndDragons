#include "zobristHash.h"

ZobristHash::ZobristHash() {
  srand (time(NULL));
  for (int i=0;i<_H*_W;i++) {
    for (int j=0;j<_N;j++) {
      table[i][j] = rand();
    }
  }
}

int ZobristHash::hash(int* board) {
  int val = 0;
  for (int i=0;i<_H*_W;i++) {
    if (board[i] != 0) {
      int j = board[i];
      val ^= table[i][j];
    }
  }
  return val;
}
