//
// Created by Valentin Lhermitte on 18/11/2025.
//

#ifndef FPGA_PLACER_H
#define FPGA_PLACER_H

#include "types.h"

#include <vector>
#include <string>

#define TIME_LIMIT 235.0 // seconds

struct BlockSortEntry {
    int block_idx;
    double ideal_x;
    double ideal_y;
};

/**
 * @class Placer
 * @brief Manages the placement of blocks and calculation of metrics.
 */
class Placer {
public:
    /**
     * @brief Constructs a Placer object with the given circuit.
     * @param circuit The circuit to be placed (moved).
     */
    explicit Placer(Circuit circuit);

    /**
     * @brief Runs the main placement algorithm.
     *
     * THIS IS THE FUNCTION YOU WILL MODIFY with a real algorithm
     * (e.g., Simulated Annealing).
     */
    void place();

    /**
     * @brief Calculates the total Half-Perimeter Wirelength (HPWL)
     * of the current placement.
     * @return The total HPWL.
     */
    double calculate_total_hpwl() const;

    /**
     * @brief Calculates the Congestion Coefficient (CC) of the
     * current placement.
     * @return The Congestion Coefficient.
     */
    double calculate_congestion_coefficient();

    /**
     * @brief Prints the current placement grid to the console.
     * For debugging purposes.
     */
    void print_placement_grid() const;

    /**
     * @brief Writes the final placement to the specified output file.
     * @param output_filename The path to the output file.
     */
    void write_output(const std::string& output_filename) const;

    Circuit get_circuit() const { return circuit_; }

private:
    // --- Initial Placement Methods ---
    /**
     * @brief Performs a random legal initial placement.
     */
    void random_initial_place();

    // --- Auxiliary Functions (Cost, Calibration, Updates) ---
    /**
     * @brief Calibrates the initial temperature that ensure a high acceptance rate (~95%).
     * @return The calibrated temperature.
     */
    double calibrate_T(double target_acceptance_rate, double lambda);

    /**
     * @brief Updates the congestion map and cost structures
     * for the specified nets.
     * @param net_indices The indices of the nets to update.
     * @param remove If true, decrements congestion; if false, increments.
     */
    void update_cost_structures(const std::vector<int>& net_indices, bool remove);

    /**
     * @brief Gets the bounding box for a single net.
     * @param net The net to evaluate.
     * @return The BoundingBox for the net.
     */
    BoundingBox get_net_bbox(const Net& net) const;

    void initialize_bbox_cache();

    void update_bbox_cache(int net_idx, int x1, int y1, int x2, int y2, bool is_swap);

    void update_single_net_bbox(int net_idx, int old_x, int old_y, int new_x, int new_y);

    // --- Private variables ---
    Circuit circuit_;
    int R_, C_;

    // 2D grid to check for placement legality (stores block name)
    std::vector<std::vector<int>> grid_; // EMPTY_BLOCK_ID (-1) means empty cell

    // 2D grid for congestion map (U[x,y])
    std::vector<std::vector<int>> congestion_map_U_;

    std::vector<BoundingBox> net_bboxes_;

    // Mapping from block index to the list of net indices it belongs to
    std::vector<std::vector<int>> block_to_nets_; // Maps block_idx -> list of net_indices
    double current_hpwl_;
    double current_sum_U_;
    double current_sum_sq_U_;
};

#endif //FPGA_PLACER_H
