#ifndef SEARCH_H
#define SEARCH_H
#include "bits/stdc++.h"
#include "board.h"
#include "params.h"
#include "zobristHash.h"
using namespace std;

class Search {
public:
  Search(int* _start, int _max_depth, int _beam);
  Search(string path, int _max_depth, int _beam);
  Board* beam_search();
  Board* beam_search(int size);
// private:
  int max_depth, beam;
  int* start;
  vector<Board*> get_all_next_states(Board* board);
  int* load_from_file(string path);
  class BoardCompare {
  public:
    bool operator() (Board* b1, Board* b2) {
      return *b1 < *b2;
    }
  };
private:
  priority_queue<Board*, vector<Board*>, BoardCompare> good_states_pq;
  set<int> visited_states;
  ZobristHash zh;
  void worker(int id, int size);
  mutex visited_mutex, good_mutex;
};

#endif
