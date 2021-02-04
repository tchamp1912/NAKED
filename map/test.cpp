#include <iostream>
#include <assert.h>
#include "MapCoder.h"
#include "Map.h"

int main() {
    MapCoder coder;
    Map testMap;
    float gridEdgeLen = 0.5;
    Cell cell1 = Cell(1, 2);
    Cell cell2 = Cell(3, 4);
    Cell cell3 = Cell(5, 6);
    Cell cell4 = Cell(7, 8);
    Chunk chunk1 = Chunk(cell1, cell2);
    Chunk chunk2 = Chunk(cell3, cell4);
    Chunk chunk3 = Chunk(cell4, cell1); 

    testMap.gridEdgeLength = gridEdgeLen;
    testMap.blockedCells.push_back(cell1);
    testMap.blockedCells.push_back(cell2);
    void* bitstream = coder.encode(testMap);
    Map decodedMap = coder.decode(bitstream); 

    assert(decodedMap.gridEdgeLength == gridEdgeLen);
    assert(decodedMap.blockedCells[0].x == cell1.x);
    assert(decodedMap.blockedCells[0].y == cell1.y);
    assert(decodedMap.blockedCells[1].x == cell2.x);
    assert(decodedMap.blockedCells[1].y == cell2.y);

    std::cout << "\033[1;32mMap encode/decode success\033[0m\n";
}