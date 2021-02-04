#ifndef MAPCODER
#define MAPCODER

// Responsible for encoding/decoding to/from the struct found in Map.h 

#include "Map.h"

class MapCoder {
    public:
        void* encode(Map map) {
            void* bitstream = new char[406]; // 406 == 101x101 QR code max bytes | TODO don't allocate a hard number
            float* grid_edge_length = (float*)bitstream;
            grid_edge_length[0] =  map.grid_edge_length;
            return bitstream;
        }

        Map decode(void* bitstream) {
            Map map;
            map.grid_edge_length = *(float*)bitstream;
            
            
            return map;
        }

};

#endif