#include <limits.h>
#include <list>
#include <cmath>

#include "Pathfinder.h"
#include "../map/Map.h"

Pathfinder::Pathfinder(const Map& map) {
  int minX = INT_MAX;
  int minY = INT_MAX;

  int maxX = INT_MIN;
  int maxY = INT_MIN;

  
  for (int i = 0; i < map.blockedChunks.size(); i++) {
    Chunk curChunk = map.blockedChunks.at(i);

    if (curChunk.topLeft.x < minX) {
      minX = curChunk.topLeft.x;
    }

    if (curChunk.bottomRight.x > maxX) {
      maxX = curChunk.bottomRight.x;
    }

    if (curChunk.bottomRight.y < minY) {
      maxY = curChunk.bottomRight.y;
    }

    if (curChunk.topLeft.y > maxY) {
      maxY = curChunk.topLeft.y;
    }
  }

  
  for (int i = 0; i < map.blockedCells.size(); i++) {
    Cell curCell = map.blockedCells.at(i);

    if (curCell.x < minX) {
      minX = curCell.x;
    }

    if (curCell.x > maxX) {
      maxX = curCell.x;
    }

    if (curCell.y < minY) {
      minY = curCell.y;
    }

    if (curCell.y > maxY) {
      maxY = curCell.y;
    }
  }

  
  for (int i = 0; i < map.qrCodes.size(); i++) {
    Cell curCell = map.qrCodes.at(i);

    if (curCell.x < minX) {
      minX = curCell.x;
    }

    if (curCell.x > maxX) {
      maxX = curCell.x;
    }

    if (curCell.y < minY) {
      minY = curCell.y;
    }

    if (curCell.y > maxY) {
      maxY = curCell.y;
    }
  }

  
  for (int i = 0; i < map.targets.size(); i++) {
    TargetCell curCell = map.targets.at(i);

    if (curCell.x < minX) {
      minX = curCell.x;
    }

    if (curCell.x > maxX) {
      maxX = curCell.x;
    }

    if (curCell.y < minY) {
      minY = curCell.y;
    }

    if (curCell.y > maxY) {
      maxY = curCell.y;
    }
  }

  this->xLength = maxX - minX + 1;
  this->yLength = maxY - minY + 1;

  this->xShift = -minX;
  this->yShift = -minY;

  // Initialize the 2D array
  this->cellMap = new CellType*[xLength];
  for (int i = 0; i < xLength; i++) {
    cellMap[i] = new CellType[yLength];
  }

  // Set the default CellType to Unblocked
  for (int x = 0; x < xLength; x++) {
    for (int y = 0; y < yLength; y++) {
      cellMap[x][y] = Unblocked;
    }
  }

  // Set the source cell
  cellMap[xShift][yShift] = Source;

  // Set the blocked cells
  for (int i = 0; i < map.blockedCells.size(); i++) {
    Cell blockedCell = map.blockedCells.at(i);
    cellMap[blockedCell.x + xShift][blockedCell.y + yShift] = Blocked;
  }

  // Uncompress the blocked chunks
  for (int i = 0; i < map.blockedChunks.size(); i++) {
    Chunk blockedChunk = map.blockedChunks.at(i);
    for (int x = blockedChunk.topLeft.x + xShift; x <= blockedChunk.bottomRight.x + xShift; x++) {
      for (int y = blockedChunk.bottomRight.y + yShift; y <= blockedChunk.topLeft.y + yShift; y++) {
        cellMap[x][y] = Blocked;
      }
    }
  }

  // Set the QR Code locations
  for (int i = 0; i < map.qrCodes.size(); i++) {
    Cell qrCode = map.qrCodes.at(i);
    cellMap[qrCode.x + xShift][qrCode.y + yShift] = Qr_Code;
  }

  // Set the target locations
  for (int i = 0; i < map.targets.size(); i++) {
    TargetCell target = map.targets.at(i);
    cellMap[target.x + xShift][target.y + yShift] = Target;
  }

  
}

std::vector<Cell>& Pathfinder::findPathToCell(const Cell& sourceCell, const Cell& targetCell) {
  // Initialize the open list
  std::vector<CellNode&> openList;

  // Initialize the closed list
  std::vector<CellNode&> closedList;
  // bool closedList[xLength][yLength];
  // memset(closedList, false, sizeof(closedList));

  std::vector<Cell>* pathCoordinates;
  pathCoordinates->push_back(sourceCell);

  // put the starting node on the open list
  CellNode sourceNode;
  sourceNode.parent = nullptr;
  sourceNode.cell = sourceCell;
  sourceNode.g = 0;
  sourceNode.h = findDistance(sourceCell, targetCell);
  sourceNode.f = sourceNode.g + sourceNode.h;

  CellNode& sourceNodeRef = sourceNode;

  openList.push_back(sourceNode);

  // while the open list is not empty
  while(!openList.empty()) {

    // find the node with the least f on the open list, call it "q"
    CellNode& nodeQ = openList.back();

    // pop q off the open list
    openList.pop_back();

    // generate successor cells
    Cell northCell{nodeQ.cell.x, nodeQ.cell.y + 1};
    Cell southCell{nodeQ.cell.x, nodeQ.cell.y - 1};
    Cell westCell{nodeQ.cell.x - 1, nodeQ.cell.y};
    Cell eastCell{nodeQ.cell.x + 1, nodeQ.cell.y};

    if (isCellOnMap(northCell)) {
      if (!isCellBlocked(northCell)) {
        CellNode north;
        north.parent = &nodeQ;
        north.cell = northCell;
        north.g = nodeQ.g + 1;
        north.h = findDistance(northCell, targetCell);
        north.f = north.g + north.h;

        if (north.cell.equals(targetCell)) {
          return generateCoordinateList(north);
        }

        if (!isThereASmallerF(north, openList)) {
          if (!isThereASmallerF(north, closedList)) {
            insertBasedOnF(north, openList);
          }
        }
      }
    }

    if (isCellOnMap(southCell)) {
      if (!isCellBlocked(southCell)) {
        CellNode south;
        south.parent = &nodeQ;
        south.cell = southCell;
        south.g = nodeQ.g + 1;
        south.h = findDistance(southCell, targetCell);
        south.f = south.g + south.h;

        if (south.cell.equals(targetCell)) {
          return generateCoordinateList(south);
        }

        if (!isThereASmallerF(south, openList)) {
          if (!isThereASmallerF(south, closedList)) {
            insertBasedOnF(south, openList);
          }
        }
      }    
    }

    if (isCellOnMap(eastCell)) {
      if (!isCellBlocked(eastCell)) {
        CellNode east;
        east.parent = &nodeQ;
        east.cell = eastCell;
        east.g = nodeQ.g + 1;
        east.h = findDistance(eastCell, targetCell);
        east.f = east.g + east.h;

        if (east.cell.equals(targetCell)) {
          return generateCoordinateList(east);
        }

        if (!isThereASmallerF(east, openList)) {
          if (!isThereASmallerF(east, closedList)) {
            insertBasedOnF(east, openList);
          }
        } 
      }    
    }

    if (isCellOnMap(westCell)) {
      if (!isCellBlocked(westCell)) {
        CellNode west;
        west.parent = &nodeQ;
        west.cell = westCell;
        west.g = nodeQ.g + 1;
        west.h = findDistance(westCell, targetCell);
        west.f = west.g + west.h;

        if (west.cell.equals(targetCell)) {
          return generateCoordinateList(west);
        }

        if (!isThereASmallerF(west, openList)) {
          if (!isThereASmallerF(west, closedList)) {
            insertBasedOnF(west, openList);
          }
        }
      }    
    }
    closedList.push_back(nodeQ);
  }
}

void insertBasedOnF(CellNode& node, std::vector<CellNode&>& list) {
  for (std::vector<CellNode&>::iterator it; it != list.end(); ++it) {
    if (node.f > it->f) {
      list.insert(it, node);
      return;
    }
  }
  list.push_back(node);
}

int Pathfinder::findDistance(const Cell& source, const Cell& target) {
  return std::abs(source.x - target.x) + std::abs(source.y - target.y);
}

bool Pathfinder::isThereASmallerF(const CellNode node, std::vector<CellNode&>& list) {
  for (int i = 0; i < list.size(); i++) {
    if (list.at(i).cell.equals(node.cell) && list.at(i).f < node.f) {
      return true;
    }
  }
  return false;
}

bool Pathfinder::isCellOnMap(const Cell& cell) {
  return cell.x >= 0 && cell.y >= 0 && cell.x < xLength && cell.y < yLength;
}

bool Pathfinder::isCellBlocked(const Cell& cell) {
  return isCellOnMap(cell) && cellMap[cell.x][cell.y] == Blocked;
}

std::vector<Cell>& Pathfinder::generateCoordinateList(const CellNode& node) {
  std::vector<Cell> coordinateList;

  CellNode curNode = node;

  do {
    coordinateList.push_back(curNode.cell);
    curNode = *curNode.parent;
  } while(node.parent != nullptr);

  return coordinateList;
}