#ifndef MAP
#define MAP

#include <vector>

struct Target {
    short x;
    short y;
    short instruction;
};

struct Cell {
    short x;
    short y;
};

struct Chunk {
    Cell top_left;
    Cell bottom_right;
};

struct Map {
    float grid_edge_length;             // In meters
    std::vector<Chunk> blocked_chunks;
    std::vector<Cell> blocked_cells;    
    std::vector<Target> targets;        // Sanitization targets
    std::vector<Cell> qr_codes;         // Location of other codes in environment
};



#endif