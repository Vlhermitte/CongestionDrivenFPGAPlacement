//
// Created by Valentin Lhermitte on 18/11/2025.
//

#ifndef FPGA_TYPES_H
#define FPGA_TYPES_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>


// Represents a movable logic block.
// Its coordinates (x, y) are integers.
struct Block {
    std::string name;
    int x = -1; // Initial invalid coordinates
    int y = -1;
};

// Represents a fixed I/O pin.
// Its coordinates (x, y) can be non-integers.
struct Pin {
    std::string name;
    double x;
    double y;
};

// Represents a terminal, which can be either a block or a pin.
// We use a simple enum and string name for simplicity.
enum class TerminalType { BLOCK, PIN };

struct Terminal {
    std::string name;
    TerminalType type;
};

// Represents a net, which is a collection of terminals.
struct Net {
    std::string name;
    std::vector<Terminal> terminals;
};

// Bounding box definition, using doubles to accommodate pins.
struct BoundingBox {
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
};

// Holds all data for the circuit.
struct Circuit {
    int R = 0; // Rows
    int C = 0; // Columns

    std::vector<Block> blocks;
    std::vector<Pin> pins;
    std::vector<Net> nets;

    // Maps for fast lookups
    std::unordered_map<std::string, int> block_name_to_index;
    std::unordered_map<std::string, int> pin_name_to_index;
};


#endif //FPGA_TYPES_H