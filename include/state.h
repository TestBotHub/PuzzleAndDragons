#ifndef STATE_H
#define STATE_H

#include "board.h"
#include <bits/stdc++.h>

using namespace std;

class State {
  public:
    vector<Board> getAllNextStates();
    /**
     * Based on path, visualize route of the drop on a board.
     */
    void visualize_path();
  private:
};

#endif
