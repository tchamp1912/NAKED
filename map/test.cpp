#include <iostream>
#include <assert.h>
#include "MapCoder.h"
#include "Map.h"

int main() {
    MapCoder coder;
    Map testMap;
    float grid_edge_len = 0.5;


    testMap.grid_edge_length = grid_edge_len;
    void* bitstream = coder.encode(testMap);
    Map decodedMap = coder.decode(bitstream); 

    assert(decodedMap.grid_edge_length == grid_edge_len);

    std::cout << "\033[1;32mMap encode/decode success\033[0m\n";
}