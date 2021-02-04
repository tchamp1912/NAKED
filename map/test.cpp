#include <iostream>
#include <assert.h>
#include "MapCoder.h"
#include "Map.h"

int main() {
    // Declare dummy values
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
    Target target1 = Target(10, 20, 30);
    Target target2 = Target(11, 21, 31);
    // Load up the map w/ dummy data
    testMap.gridEdgeLength = gridEdgeLen;
    testMap.blockedCells.push_back(cell1);
    testMap.blockedCells.push_back(cell2);
    testMap.blockedChunks.push_back(chunk1);
    testMap.blockedChunks.push_back(chunk2);
    testMap.blockedChunks.push_back(chunk3);
    testMap.targets.push_back(target1);
    testMap.targets.push_back(target2);
    testMap.qrCodes.push_back(cell4);
    testMap.qrCodes.push_back(cell1);
    void* bitstream = coder.encode(testMap);
    Map decodedMap = coder.decode(bitstream); 
    // Assure equality
    assert(decodedMap.equals(testMap));
    std::cout << "\033[1;32mMap encode/decode success\033[0m\n";
}