#ifndef MAP
#define MAP

#include <vector>

struct Target {
    Target(short x_, short y_, short instruction_) {
        x = x_;
        y = y_;
        instruction = instruction_;
    }

    short x;
    short y;
    short instruction;
};

struct Cell {
    Cell(short x_, short y_) {
        x = x_;
        y = y_;
    }

    short x;
    short y;
};

struct Chunk {
    Chunk(Cell tl, Cell br) {
        topLeft = tl;
        bottomRight = br;
    }
    
    Cell topLeft;
    Cell bottomRight;
};

struct Map {
    float gridEdgeLength;             // In meters
    std::vector<Cell> blockedCells;    
    std::vector<Chunk> blockedChunks;
    std::vector<Target> targets;        // Sanitization targets
    std::vector<Cell> qrCodes;         // Location of other codes in environment
};



#endif