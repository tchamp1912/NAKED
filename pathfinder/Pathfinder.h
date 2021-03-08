#ifndef PATHFINDER
#define PATHFINDER

#include <vector>
#include "../map/Map.h"

class Pathfinder {
  private: 
    CellType** cellMap;
    int xLength;
    int yLength;
    int xShift;
    int yShift;

  public:
    Pathfinder(const Map& map);
    CellType&& findPathToCell(const Cell& sourceCell, const Cell& targetCell);

  private:
    void insertBasedOnF(CellNode& node, std::vector<CellNode>& vector);
    int findDistance(const Cell& source, const Cell& target);
    bool isThereASmallerF(int f, const std::vector<CellNode>& vector);
    bool isCellOnMap(const Cell& cell);
    bool isCellBlocked(const Cell& cell);
};

enum CellType { Blocked, Unblocked, Target, Source, Qr_Code};

struct CellNode {
  Cell parent;
  Cell cell;
  int f;
  int g;
  int h;
};

#endif