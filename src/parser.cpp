//
// Created by Valentin Lhermitte on 18/11/2025.
//

#include "parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

Circuit Parser::parse(const std::string& input_filename) {
    std::ifstream file(input_filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open input file: " + input_filename);
    }

    Circuit circuit;
    std::string line;
    std::string token;

    // 1. Read Header
    int num_logic_blocks, num_io_pins, num_nets;
    if (!std::getline(file, line)) {
        throw std::runtime_error("Failed to read header line.");
    }
    std::stringstream ss_header(line);
    ss_header >> circuit.R >> circuit.C >> num_logic_blocks >> num_io_pins >> num_nets;

    if (circuit.R <= 0 || circuit.C <= 0) {
        throw std::runtime_error("Invalid grid dimensions.");
    }

    // 2. Read Movable Logic Blocks
    circuit.blocks.reserve(num_logic_blocks);
    for (int i = 0; i < num_logic_blocks; ++i) {
        if (!std::getline(file, line)) {
            throw std::runtime_error("Unexpected end of file while reading logic blocks.");
        }
        std::stringstream ss_block(line);
        std::string block_name;
        ss_block >> block_name;
        circuit.block_name_to_index[block_name] = i;
        circuit.blocks.push_back({i, block_name, -1, -1});
    }

    // 3. Read Fixed I/O Pins
    circuit.pins.reserve(num_io_pins);
    for (int i = 0; i < num_io_pins; ++i) {
        if (!std::getline(file, line)) {
            throw std::runtime_error("Unexpected end of file while reading I/O pins.");
        }
        std::stringstream ss_pin(line);
        std::string pin_name;
        double x, y;
        ss_pin >> pin_name >> x >> y;
        circuit.pin_name_to_index[pin_name] = i;
        circuit.pins.push_back({i, pin_name, x, y});
    }

    // 4. Read Nets
    circuit.nets.reserve(num_nets);
    for (int i = 0; i < num_nets; ++i) {
        if (!std::getline(file, line)) {
            throw std::runtime_error("Unexpected end of file while reading nets.");
        }
        std::stringstream ss_net(line);
        Net net;
        int degree;
        ss_net >> net.name >> degree;
        net.terminals.reserve(degree);

        for (int j = 0; j < degree; ++j) {
            std::string terminal_name;
            ss_net >> terminal_name;
            if (circuit.block_name_to_index.contains(terminal_name)) {
                int id = circuit.block_name_to_index[terminal_name];
                net.terminals.push_back({id, TerminalType::BLOCK});
            } else if (circuit.pin_name_to_index.contains(terminal_name)) {
                int id = circuit.pin_name_to_index[terminal_name];
                net.terminals.push_back({id, TerminalType::PIN});
            } else {
                std::cerr << "Warning: Terminal '" << terminal_name << "' in net '" << net.name << "' not found as block or pin." << std::endl;
            }
        }
        circuit.nets.push_back(std::move(net));
    }

    file.close();
    return circuit;
}