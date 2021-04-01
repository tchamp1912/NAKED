#include <iostream>
#include <assert.h>
#include "MapCoder.h"
#include "Map.h"

void simple() {
    MapCoder coder;
    Map testMap;
    float gridEdgeLen = 1.0;
    Cell tl = Cell(-2, 4);
    Cell tr = Cell(2, 4);
    Cell bl = Cell(-2, -2);
    Cell br = Cell(2, 4);
    Chunk left = Chunk(tl, bl);
    Chunk top = Chunk(tl, tr);
    Chunk right = Chunk(tr, br);
    Chunk bottom = Chunk(bl, br);
    Target target1 = Target(0, 2, 30);
    // Load up the map w/ dummy data
    testMap.gridEdgeLength = gridEdgeLen;
    testMap.blockedChunks.push_back(left);
    testMap.blockedChunks.push_back(top);
    testMap.blockedChunks.push_back(right);
    testMap.blockedChunks.push_back(bottom);
    testMap.targets.push_back(target1);
    coder.generateQr(coder.encode(testMap));
}


int main() {
    // NOTE: Ensure all functions except for desired QR are commented. Otherwise you may overwrite
    simple();
}