//
// Created by Valentin Lhermitte on 18/11/2025.
//

#include <iostream>
#include <string>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <thread>

#include "parser.h"
#include "placer.h"


/**
 * @brief Main entry point for the placer.
 *
 * Usage: ./placer <input_file> <output_file>
 */
int main(int argc, char* argv[]) {
    // Check for correct command-line arguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << std::endl;
        return 1;
    }

    std::string input_filename = argv[1];
    std::string output_filename = argv[2];

    // Chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    // int num_threads = std::thread::hardware_concurrency();

    try {
        // 1. Parse the input file
        std::cerr << "Parsing input file: " << input_filename << std::endl;
        Circuit circuit = Parser::parse(input_filename);
        std::cerr << "Parsing complete." << std::endl;
        std::cerr << "Grid: " << circuit.R << "x" << circuit.C << std::endl;
        std::cerr << "Blocks: " << circuit.blocks.size() << ", Pins: " << circuit.pins.size() << ", Nets: " << circuit.nets.size() << std::endl;

        // 2. Create the Placer and run the placement algorithm
        Placer placer(std::move(circuit));
        std::cerr << "Starting placement..." << std::endl;
        placer.place();
        std::cerr << "Placement complete." << std::endl;

        // 3. Calculate and report final metrics
        double final_hpwl = placer.calculate_total_hpwl();
        double final_cc = placer.calculate_congestion_coefficient();

        double avg_hpwl_per_net = final_hpwl / placer.get_circuit().nets.size();

        // placer.print_placement_grid();

        std::cerr << "--------------------------------" << std::endl;
        std::cerr << "Final Total HPWL: " << final_hpwl << std::endl;
        std::cerr << "  - Average HPWL per Net: " << avg_hpwl_per_net << std::endl;
        std::cerr << "Final Congestion (CC): " << final_cc << std::endl;
        std::cerr << "--------------------------------" << std::endl;


        // 4. Write the legal placement to the output file
        std::cerr << "Writing output to: " << output_filename << std::endl;
        placer.write_output(output_filename);
        std::cerr << "Done." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cerr << "Total execution time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}