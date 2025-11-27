//
// Created by Valentin Lhermitte on 18/11/2025.
//

#ifndef FPGA_PARSER_H
#define FPGA_PARSER_H

#include <string>
#include "types.h"


/**
 * @class Parser
 * @brief Handles parsing of the input file into a Circuit data structure.
 */
class Parser {
public:
    /**
     * @brief Parses the input file and returns a complete Circuit object.
     * @param input_filename The path to the input file.
     * @return A Circuit object populated with data from the file.
     */
    static Circuit parse(const std::string& input_filename);
};

#endif //FPGA_PARSER_H