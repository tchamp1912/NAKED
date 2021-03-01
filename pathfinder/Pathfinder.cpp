#include <limits.h>

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

  int xLength = maxX - minX + 1;
  int yLength = maxY - minY + 1;

  int xShift = -minX;
  int yShift = -minY;

  // Initialize the 2D array
  cellMap = new CellType*[xLength];
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