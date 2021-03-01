#ifndef MAP
#define MAP

#include <vector>



struct Cell {
    Cell() { }

    Cell(short x_, short y_) {
        x = x_;
        y = y_;
    }

    short x;
    short y;

    bool equals(Cell other) {
        return x == other.x && y == other.y;
    }
};

struct Chunk {
    Chunk(short tl_x, short tl_y, short br_x, short br_y) {
        Cell tl(tl_x, tl_y);
        Cell br(br_x, br_y);
        topLeft = tl;
        bottomRight = br;
    }

    Chunk(Cell tl, Cell br) {
        topLeft = tl;
        bottomRight = br;
    }
    
    Cell topLeft;
    Cell bottomRight;

    bool equals(Chunk other) {
        return topLeft.equals(other.topLeft) && bottomRight.equals(other.bottomRight);
    }
};

struct Targeta {
    Targeta(short x_, short y_, short instruction_) {
        x = x_;
        y = y_;
        instruction = instruction_;
    }

    short x;
    short y;
    short instruction;

    bool equals(Targeta other) {
        return x == other.x && y == other.y && instruction == other.instruction;
    }
};

struct Map {
    float gridEdgeLength;               // In meters
    std::vector<Cell> blockedCells;    
    std::vector<Chunk> blockedChunks;
    std::vector<Targeta> targets;        // Sanitization targets
    std::vector<Cell> qrCodes;          // Location of other codes in environment

    bool equals(Map other) {
        if (gridEdgeLength != other.gridEdgeLength
            || blockedCells.size() != other.blockedCells.size()
            || blockedChunks.size() != other.blockedChunks.size()
            || targets.size() != other.targets.size()
            || qrCodes.size() != other.qrCodes.size()) return false;
        for (int i = 0; i < blockedCells.size(); i++)
            if (!blockedCells[i].equals(other.blockedCells[i])) return false;
        for (int i = 0; i < blockedChunks.size(); i++)
            if (!blockedChunks[i].equals(other.blockedChunks[i])) return false;
        for (int i = 0; i < targets.size(); i++)
            if (!targets[i].equals(other.targets[i])) return false;
        for (int i = 0; i < targets.size(); i++)
            if (!qrCodes[i].equals(other.qrCodes[i])) return false;
        return true;
    }
};



#endif