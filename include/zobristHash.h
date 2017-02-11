#ifndef ZOBRIST_HASH_H
#define ZOBRIST_HASH_H
#include "bits/stdc++.h"
#include "params.h"
using namespace std;
class ZobristHash {
public:
  int table[_H*_W][_N];
  ZobristHash();
  int hash(int* board);
};
#endif
