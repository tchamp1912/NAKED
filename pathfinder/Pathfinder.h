#ifndef PATHFINDER
#define PATHFINDER

#include <vector>
#include "../map/Map.h"

enum CellType { Blocked, Unblocked, Target, Source, Qr_Code};

class Pathfinder {
  private: 
    CellType** cellMap;

  public:
    Pathfinder(const Map& map);

    CellType&& findPathToCell(const Cell& targetCell);
};

#endif