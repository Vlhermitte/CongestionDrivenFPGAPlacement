//
// Created by Valentin Lhermitte on 18/11/2025.
//

#include "placer.h"

#include <stdexcept>
#include <fstream>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <random>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <numeric>


Placer::Placer(Circuit circuit)
    : circuit_(std::move(circuit)),
      R_(circuit_.R),
      C_(circuit_.C), current_hpwl_(0), current_sum_U_(0), current_sum_sq_U_(0)
{
    if (R_ <= 0 || C_ <= 0)
    {
        throw std::invalid_argument("Grid dimensions must be positive.");
    }

    // Initialize the grid representations
    grid_.resize(R_, std::vector<std::string>(C_, ""));
    congestion_map_U_.resize(R_, std::vector<int>(C_, 0));
    net_bboxes_.resize(circuit_.nets.size());

    block_to_nets_.resize(circuit_.blocks.size());
    for (size_t i = 0; i < circuit_.nets.size(); ++i)
    {
        const auto& net = circuit_.nets[i];
        for (const auto& term : net.terminals)
        {
            if (term.type == TerminalType::BLOCK)
            {
                int blk_idx = circuit_.block_name_to_index.at(term.name);
                // Avoid duplicates if a net connects to multiple pins on the same block
                auto& list = block_to_nets_.at(blk_idx);
                if (std::find(list.begin(), list.end(), i) == list.end())
                {
                    list.push_back(i);
                }
            }
        }
    }
}

// --- Main Placement Function ---

void Placer::random_initial_place() {
    std::cerr << "Performing random initial placement..." << std::endl;

    std::random_device rd;
    std::mt19937 rng(rd());
    // std::mt19937 rng(42); // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist_r(0, R_ - 1);
    std::uniform_int_distribution<int> dist_c(0, C_ - 1);

    for (auto& block : circuit_.blocks) {
        int x, y;
        do {
            x = dist_c(rng);
            y = dist_r(rng);
        } while (!grid_[y][x].empty()); // Ensure legality

        block.x = x;
        block.y = y;
        grid_[y][x] = block.name;
    }
}


void Placer::place() {
    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Initial placement
    random_initial_place();

    // 2. Setup RNG
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist_block_idx(0, circuit_.blocks.size() - 1);
    std::uniform_real_distribution<double> dist_prob(0.0, 1.0);

    // 3. SA Parameters and Initial Cost Calculation

    // Initialize the incremental cost trackers before the loop starts
    // Calculate initial full congestion map
    calculate_congestion_coefficient(); // Populates congestion_map_U_ initially

    // Initialize sums from the map
    current_sum_U_ = 0;
    current_sum_sq_U_ = 0;
    for(int y = 0; y < R_; ++y) {
        for(int x = 0; x < C_; ++x) {
            double u = congestion_map_U_[y][x];
            current_sum_U_ += u;
            current_sum_sq_U_ += (u * u);
        }
    }
    // Initialize HPWL
    current_hpwl_ = calculate_total_hpwl();

    // Calculate initial cost using the new O(1) variables
    double cc_initial = 1.0;
    if (current_sum_U_ != 0) {
        double N = R_ * C_;
        double avg_U_sq = (current_sum_U_ / N) * (current_sum_U_ / N);
        cc_initial = (current_sum_sq_U_ / N) / avg_U_sq;
    }

    // Dynamic lambda schedule
    double lambda_min = (current_hpwl_ / cc_initial) * 0.1;    // Prioritize HPWL early
    double lambda_max = (current_hpwl_ / cc_initial);   // Prioritize Congestion late
    double lambda = lambda_min;   // Start low

    double current_cost = current_hpwl_ + lambda * cc_initial;

    // Initial temperature (Heuristic: usually set to allow ~99% acceptance initially)
    // Here we start high enough to escape local minima.
    double T = calibrate_T(0.95, lambda);
    // double T = current_cost * 0.01; // Simple heuristic
    double T_initial = T;
    double T_final = 0.005;

    // Moves per temperature step
    // Typically proportional to the number of blocks (e.g., 10 * N)
    int moves_per_temp = 10 * circuit_.blocks.size();

    double range_limiter = 1.0;

    std::cout << "Starting SA..." << std::endl;
    std::cout << "Initial Cost: " << current_cost << " (T=" << T << ")" << std::endl;

    for (size_t i = 0; i < circuit_.nets.size(); ++i) {
        net_bboxes_[i] = get_net_bbox(circuit_.nets[i]);
    }

    std::vector<int> affected_nets;
    affected_nets.reserve(100); // Reserve decent space

    // ---------------------------- //
    // 4. MAIN ANNEALING LOOP       //
    // ---------------------------- //
    while (T > T_final) {
        int accepted_moves = 0;

        for (int i = 0; i < moves_per_temp; ++i) {
            affected_nets.clear();

            if ((i & 1023) == 0) {
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = now - start_time;
                if (elapsed.count() > TIME_LIMIT) {
                    std::cout << "\n!!! Time Limit Reached (" << elapsed.count() << "s). Stopping early. !!!" << std::endl;
                    goto end_annealing; // Jump out of both loops
                }
            }

            // -------------- A. PROPOSE MOVE -------------- //
            // -------------- A. SELECT MOVE STRATEGY VIA RL -------------- //

            // --- Dynamic Window Calculation ---
            // Calculate range based on the feedback controller variable 'range_limiter'
            int range_w = std::max(1, (int)(C_ * range_limiter));
            int range_h = std::max(1, (int)(R_ * range_limiter));

            int src_idx = dist_block_idx(rng);
            Block& src_block = circuit_.blocks[src_idx];
            int x1 = src_block.x;
            int y1 = src_block.y;

            std::uniform_int_distribution dist_dx(-range_w, range_w);
            std::uniform_int_distribution dist_dy(-range_h, range_h);

            int x2 = x1 + dist_dx(rng);
            int y2 = y1 + dist_dy(rng);

            x2 = std::max(0, std::min(C_ - 1, x2));
            y2 = std::max(0, std::min(R_ - 1, y2));

            // Clamp to grid boundaries
            x2 = std::max(0, std::min(C_ - 1, x2));
            y2 = std::max(0, std::min(R_ - 1, y2));

            // Skip if source and dest are the same
            if (x1 == x2 && y1 == y2) continue;

            // Check what is at the destination
            std::string dest_name = grid_[y2][x2];
            int dest_idx = -1;
            bool is_swap = false;

            if (!dest_name.empty()) {
                // There is a block at the destination -> SWAP
                is_swap = true;
                dest_idx = circuit_.block_name_to_index.at(dest_name);
            }

            // Identify nets and remove their cost contributions
            affected_nets = block_to_nets_.at(src_idx);
            if (is_swap) {
                const auto& dest_nets = block_to_nets_.at(dest_idx);
                affected_nets.insert(affected_nets.end(), dest_nets.begin(), dest_nets.end());
            }
            // Remove duplicates (crucial if source and dest are on the same net)
            std::sort(affected_nets.begin(), affected_nets.end());
            affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());

            std::vector<BoundingBox> old_bboxes;
            old_bboxes.reserve(affected_nets.size());
            for(int net_idx : affected_nets) {
                old_bboxes.push_back(net_bboxes_[net_idx]);
            }

            // REMOVE OLD COST CONTRIBUTION (Sets state to "undefined/transition")
            update_cost_structures(affected_nets, true); // true = remove

            // -------------- A. PROPOSE MOVE (end) -------------- //

            // -------------- B. PERFORM TENTATIVE SWAP/MOVE -------------- //

            // Move Source to (x2, y2)
            src_block.x = x2;
            src_block.y = y2;
            grid_[y2][x2] = src_block.name;

            // If Swap, move Dest block to (x1, y1)
            if (is_swap) {
                Block& dest_block = circuit_.blocks[dest_idx];
                dest_block.x = x1;
                dest_block.y = y1;
                grid_[y1][x1] = dest_block.name;
            } else {
                // Empty destination, so old source position becomes empty
                grid_[y1][x1] = "";
            }

            // 3. Update BBox Cache (The new function)
            for (int net_idx : affected_nets) {
                // Pass: net_index, old_src_coords, new_src_coords, is_swap
                update_bbox_cache(net_idx, x1, y1, x2, y2, is_swap);
            }

            // ADD NEW COST CONTRIBUTION (Sets state to "new potential state")
            update_cost_structures(affected_nets, false); // false = add

            // CALCULATE NEW COST (O(1) calculation)
            double cc = 1.0;
            if (current_sum_U_ != 0) {
                double N = R_ * C_;
                double avg_U_sq = (current_sum_U_ / N) * (current_sum_U_ / N);
                cc = (current_sum_sq_U_ / N) / avg_U_sq;
            }

            // -------------- B. PERFORM TENTATIVE SWAP/MOVE (end) -------------- //

            // -------------- C. EVALUATE -------------- //
            double new_cost = current_hpwl_ + lambda * cc;
            double delta_cost = new_cost - current_cost;
            // -------------- C. EVALUATE (end)-------------- //

            // -------------- D. ACCEPT OR REJECT -------------- //
            bool accept = false;
            if (delta_cost < 0) {
                accept = true; // Improvement
            } else {
                // Metropolis criterion: prob = exp(-delta / T)
                double prob = std::exp(-delta_cost / T);
                if (dist_prob(rng) < prob) {
                    accept = true;
                }
            }

            if (accept) {
                current_cost = new_cost;
                accepted_moves++;
            } else {
                // --- REVERT MOVE ---
                // 1. Remove the "New" (bad) contributions
                update_cost_structures(affected_nets, true);

                // 2. Revert positions (Same as original code)
                src_block.x = x1;
                src_block.y = y1;
                grid_[y1][x1] = src_block.name;

                if (is_swap) {
                    Block& dest_block = circuit_.blocks[dest_idx];
                    dest_block.x = x2;
                    dest_block.y = y2;
                    grid_[y2][x2] = dest_block.name;
                } else {
                    grid_[y2][x2] = "";
                }

                // We must restore bboxes *before* re-adding cost, so the cost calculation uses the correct box
                for(size_t k = 0; k < affected_nets.size(); ++k) {
                    net_bboxes_[affected_nets[k]] = old_bboxes[k];
                }

                // 3. Restore the "Old" (good) contributions
                update_cost_structures(affected_nets, false);
            }
            // -------------- D. ACCEPT OR REJECT (end) -------------- //
        }

        // -------------- E. COOL DOWN -------------- //
        // Adaptive Cooling Schedule
        double acceptance_rate = (double)accepted_moves / moves_per_temp;
        // Goal: Keep acceptance rate around 0.44
        if (accepted_moves > 0) {
            range_limiter = range_limiter * (acceptance_rate / 0.44);
        } else {
            // If acceptance is 0, we are frozen. Drastically shrink to try and find ANY valid move.
            range_limiter = std::min(1.0, range_limiter * 1.5);
        }

        // Clamp the limiter to avoid 0 range or exceeding bounds
        range_limiter = std::max(0.02, std::min(1.0, range_limiter));

        // Adaptive alpha
        double alpha;
        if (acceptance_rate > 0.95) {
            alpha = 0.5; // Only cool fast if we are basically accepting everything (random walk)
        } else if (acceptance_rate > 0.8) {
            alpha = 0.90; // High acceptance phase. Cool relatively fast.
        } else if (acceptance_rate > 0.15) {
            alpha = 0.95; // Critical phase (crystallization). Cool very slowly.
        } else {
            alpha = 0.80; // Frozen phase.
        }

        T *= alpha;
        // Logging to track progress
        std::cout << "T: " << T << " Cost: "
            << current_cost << static_cast<double>(accepted_moves)/moves_per_temp
            << " Acceptance: " << acceptance_rate * 100.0
            << "% Lambda: " << lambda
            << std::endl;

        // -------------- E. COOL DOWN (end)-------------- //

        // --- F. UPDATE DYNAMIC LAMBDA ---

        // Calculate progress (0.0 at start, -> 1.0 at end)
        // We clamp it to 1.0 just in case T drops below T_final slightly
        double progress = 1.0 - (T / T_initial);
        if (progress < 0) progress = 0;
        if (progress > 1) progress = 1;

        // Cosine Schedule: Grow lambda as T drops
        lambda = lambda_min + 0.5 * (lambda_max - lambda_min) * (1 - std::cos(M_PI * progress));

        // -------------- G. CORRECT ANY ERRORS (NUMERICAL DRIFT) -------------- //
        calculate_congestion_coefficient(); // This resets and refills congestion_map_U_ and current_sum_sq_U_

        // -------------- F. CORRECT ANY ERRORS (end) -------------- //
    }

    end_annealing:
    // Restore the Best Solution before exiting
    // std::cout << "Restoring best solution found (Cost: " << global_best_cost << ")..." << std::endl;
    // circuit_.blocks = best_blocks;
    //
    // // Restore the grid to match the best blocks
    // // Clear grid
    // for(auto& row : grid_) {
    //     std::fill(row.begin(), row.end(), "");
    // }
    // // Fill grid
    // for(const auto& b : circuit_.blocks) {
    //     grid_[b.y][b.x] = b.name;
    // }
    std::cout << "Solution found (Cost: " << current_cost << ")..." << std::endl;
}


// --- Auxiliary Functions (Cost, Calibration, Updates) ---

double Placer::calibrate_T(double target_acceptance_rate, double lambda) {
    std::cerr << "Calibrating initial temperature..." << std::endl;
    double calibrated_T = 1.0;

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist_block(0, circuit_.blocks.size() - 1);
    std::uniform_int_distribution<int> dist_r(0, R_ - 1);
    std::uniform_int_distribution<int> dist_c(0, C_ - 1);

    const int NUM_SAMPLES = 2000;
    double total_positive_delta = 0.0;
    int positive_delta_count = 0;

    // Helper to calculate cost
    auto get_current_total_cost = [&]() {
        double cc = 1.0;
        if (current_sum_U_ != 0) {
            double N = R_ * C_;
            double avg_U_sq = (current_sum_U_ / N) * (current_sum_U_ / N);
            if (avg_U_sq > 0) {
                cc = (current_sum_sq_U_ / N) / avg_U_sq;
            }
        }
        return current_hpwl_ + lambda * cc;
    };

    std::vector<int> affected_nets;
    affected_nets.reserve(200);

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        int src_idx = dist_block(rng);
        Block& src_block = circuit_.blocks[src_idx];
        int x1 = src_block.x;
        int y1 = src_block.y;

        int x2 = dist_c(rng);
        int y2 = dist_r(rng);

        if (x1 == x2 && y1 == y2) continue;

        // Check destination
        std::string dest_name = grid_[y2][x2];
        int dest_idx = -1;
        bool is_swap = false;
        if (!dest_name.empty()) {
            is_swap = true;
            dest_idx = circuit_.block_name_to_index.at(dest_name);
        }

        // 1. Identify Nets
        affected_nets.clear();
        affected_nets = block_to_nets_.at(src_idx);
        if (is_swap) {
            const auto& dest_nets = block_to_nets_.at(dest_idx);
            affected_nets.insert(affected_nets.end(), dest_nets.begin(), dest_nets.end());
        }
        std::sort(affected_nets.begin(), affected_nets.end());
        affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());

        // 2. Backup BBoxes (CRITICAL STEP MISSING IN OLD CODE)
        std::vector<BoundingBox> old_bboxes;
        old_bboxes.reserve(affected_nets.size());
        for(int net_idx : affected_nets) {
            old_bboxes.push_back(net_bboxes_[net_idx]);
        }

        // 3. Capture Old Cost
        double old_cost = get_current_total_cost();

        // 4. Apply Move (Simulate)
        update_cost_structures(affected_nets, true); // Remove old

        src_block.x = x2; src_block.y = y2;
        grid_[y2][x2] = src_block.name;

        if (is_swap) {
            Block& dest_block = circuit_.blocks[dest_idx];
            dest_block.x = x1; dest_block.y = y1;
            grid_[y1][x1] = dest_block.name;
        } else {
            grid_[y1][x1] = "";
        }

        // Update BBoxes (CRITICAL STEP MISSING IN OLD CODE)
        for (int net_idx : affected_nets) {
            update_bbox_cache(net_idx, x1, y1, x2, y2, is_swap);
        }

        update_cost_structures(affected_nets, false); // Add new

        // 5. Capture New Cost
        double new_cost = get_current_total_cost();
        double delta_cost = new_cost - old_cost;

        if (delta_cost > 0) {
            total_positive_delta += delta_cost;
            positive_delta_count++;
        }

        // 6. Revert Move
        update_cost_structures(affected_nets, true); // Remove new

        src_block.x = x1; src_block.y = y1;
        grid_[y1][x1] = src_block.name;

        if (is_swap) {
            Block& dest_block = circuit_.blocks[dest_idx];
            dest_block.x = x2; dest_block.y = y2;
            grid_[y2][x2] = dest_block.name;
        } else {
            grid_[y2][x2] = "";
        }

        // Restore BBoxes (CRITICAL STEP MISSING IN OLD CODE)
        for(size_t k = 0; k < affected_nets.size(); ++k) {
            net_bboxes_[affected_nets[k]] = old_bboxes[k];
        }

        update_cost_structures(affected_nets, false); // Add old
    }

    if (positive_delta_count > 0) {
        double avg_positive_delta = total_positive_delta / positive_delta_count;
        calibrated_T = -avg_positive_delta / std::log(target_acceptance_rate);
    } else {
        // Fallback if no moves increased cost (rare but possible)
        calibrated_T = 100.0;
    }

    std::cerr << "Calibrated initial temperature T = " << calibrated_T << " (Avg Delta: " << (total_positive_delta/positive_delta_count) << ")" << std::endl;
    return calibrated_T;
}

void Placer::update_cost_structures(const std::vector<int>& net_indices, bool remove) {
    int sign = remove ? -1 : 1;

    for (int net_idx : net_indices) {
        // --- 1. Update HPWL ---
        const BoundingBox& bbox = net_bboxes_[net_idx]; // This gets the CURRENT bbox based on block positions

        // SAFETY CHECK: If the net has no terminals, the bbox is invalid.
        // x_min initialized to DBL_MAX, x_max to -DBL_MAX.
        if (bbox.x_min >= bbox.x_max || bbox.y_min >= bbox.y_max) {
            continue; // Skip empty nets
        }

        double hpwl = (bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min);
        current_hpwl_ += sign * hpwl;

        // --- 2. Update Congestion (U and U^2) ---
        // Recalculate the covered cells for this net
        int x_start = std::max(0, std::min(C_, static_cast<int>(std::floor(bbox.x_min))));
        int y_start = std::max(0, std::min(R_, static_cast<int>(std::floor(bbox.y_min))));
        int x_end   = std::max(0, std::min(C_, static_cast<int>(std::ceil(bbox.x_max))));
        int y_end   = std::max(0, std::min(R_, static_cast<int>(std::ceil(bbox.y_max))));

        for (int y = y_start; y < y_end; ++y) {
            for (int x = x_start; x < x_end; ++x) {
                int old_u = congestion_map_U_[y][x];

                // If removing, we decrement U. If adding, we increment U.
                // We must update sum_sq_U_ based on the change: (u+1)^2 - u^2 = 2u+1, etc.

                if (remove) {
                    // Moving from old_u to (old_u - 1)
                    // Change in square: (u-1)^2 - u^2 = -2u + 1
                    current_sum_sq_U_ += (-2 * old_u + 1);
                    current_sum_U_ -= 1;
                    congestion_map_U_[y][x]--;
                } else {
                    // Moving from old_u to (old_u + 1)
                    // Change in square: (u+1)^2 - u^2 = 2u + 1
                    current_sum_sq_U_ += (2 * old_u + 1);
                    current_sum_U_ += 1;
                    congestion_map_U_[y][x]++;
                }
            }
        }
    }
}

// --- BBox Computation ---
BoundingBox Placer::get_net_bbox(const Net& net) const {
    BoundingBox bbox = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};

    for (const auto& terminal : net.terminals) {
        if (terminal.type == TerminalType::BLOCK) {
            const Block& block = circuit_.blocks[circuit_.block_name_to_index.at(terminal.name)];
            if (block.x == -1) { // Check if block is placed
                throw std::runtime_error("Calculating BBox for unplaced block: " + block.name);
            }
            // Per spec: (x_b, y_b) for min, (x_b + 1, y_b + 1) for max
            bbox.x_min = std::min(bbox.x_min, static_cast<double>(block.x));
            bbox.y_min = std::min(bbox.y_min, static_cast<double>(block.y));
            bbox.x_max = std::max(bbox.x_max, static_cast<double>(block.x + 1));
            bbox.y_max = std::max(bbox.y_max, static_cast<double>(block.y + 1));

        } else if (terminal.type == TerminalType::PIN) {
            const Pin& pin = circuit_.pins[circuit_.pin_name_to_index.at(terminal.name)];
            // Per spec: (x_p, y_p) for both min and max
            bbox.x_min = std::min(bbox.x_min, pin.x);
            bbox.y_min = std::min(bbox.y_min, pin.y);
            bbox.x_max = std::max(bbox.x_max, pin.x);
            bbox.y_max = std::max(bbox.y_max, pin.y);
        }
    }
    return bbox;
}

void Placer::initialize_bbox_cache() {
    net_bboxes_.resize(circuit_.nets.size());
    for (size_t i = 0; i < circuit_.nets.size(); ++i) {
        net_bboxes_[i] = get_net_bbox(circuit_.nets[i]);
    }
}

void Placer::update_bbox_cache(int net_idx, int x1, int y1, int x2, int y2, bool is_swap) {
    BoundingBox& bbox = net_bboxes_[net_idx];

    // Tolerance for floating point comparisons
    const double eps = 1e-9;

    // --- 1. CHECK FOR SHRINKING (Slow Path) ---
    // If a block was on the boundary, moving it *might* shrink the bbox.
    // Since we don't track *how many* blocks are on the boundary, we must recompute to be safe.

    // Check Source's OLD position (x1, y1) against current bbox
    // Blocks are 1x1, so we check min and max edges.
    bool src_on_boundary =
        (static_cast<double>(x1) <= bbox.x_min + eps) ||
        (static_cast<double>(x1 + 1) >= bbox.x_max - eps) ||
        (static_cast<double>(y1) <= bbox.y_min + eps) ||
        (static_cast<double>(y1 + 1) >= bbox.y_max - eps);

    bool dest_on_boundary = false;

    if (is_swap) {
        // Check Dest's OLD position (x2, y2)
        dest_on_boundary =
            (static_cast<double>(x2) <= bbox.x_min + eps) ||
            (static_cast<double>(x2 + 1) >= bbox.x_max - eps) ||
            (static_cast<double>(y2) <= bbox.y_min + eps) ||
            (static_cast<double>(y2 + 1) >= bbox.y_max - eps);
    }

    if (src_on_boundary || dest_on_boundary) {
        // The move involved a boundary block. We must recompute fully to ensure accuracy.
        bbox = get_net_bbox(circuit_.nets[net_idx]);
    }
    else {
        // --- 2. UPDATE EXPANSION (Fast Path: O(1)) ---
        // The blocks were internal, so the box cannot shrink.
        // We only need to check if the NEW positions expand the box.

        // Update with Source's NEW position (x2, y2)
        bbox.x_min = std::min(bbox.x_min, static_cast<double>(x2));
        bbox.x_max = std::max(bbox.x_max, static_cast<double>(x2 + 1));
        bbox.y_min = std::min(bbox.y_min, static_cast<double>(y2));
        bbox.y_max = std::max(bbox.y_max, static_cast<double>(y2 + 1));

        if (is_swap) {
            // Update with Dest's NEW position (x1, y1)
            bbox.x_min = std::min(bbox.x_min, static_cast<double>(x1));
            bbox.x_max = std::max(bbox.x_max, static_cast<double>(x1 + 1));
            bbox.y_min = std::min(bbox.y_min, static_cast<double>(y1));
            bbox.y_max = std::max(bbox.y_max, static_cast<double>(y1 + 1));
        }
    }
}

// --- Cost Calculation Functions ---
double Placer::calculate_total_hpwl() const {
    double total_hpwl = 0.0;
    for (const auto& net : circuit_.nets) {
        BoundingBox bbox = get_net_bbox(net);
        total_hpwl += (bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min);
    }
    return total_hpwl;
}

double Placer::calculate_congestion_coefficient() {
    // 1. Reset congestion map U[x,y]
    for (int y = 0; y < R_; ++y) {
        std::fill(congestion_map_U_[y].begin(), congestion_map_U_[y].end(), 0);
    }

    // 2. Compute coverage U[x,y] for all nets
    for (const auto& net : circuit_.nets) {
        BoundingBox bbox = get_net_bbox(net);

        // SAFETY CHECK: Skip empty nets
        if (bbox.x_min >= bbox.x_max || bbox.y_min >= bbox.y_max) {
            continue;
        }

        // Find integer range of sites covered by the bounding box
        // Per spec: x_min <= x < x_max AND y_min <= y < y_max
        // We need to find the integer sites [x_start, x_end) and [y_start, y_end)

        int x_start = static_cast<int>(std::floor(bbox.x_min));
        int y_start = static_cast<int>(std::floor(bbox.y_min));

        // x_max and y_max are exclusive, so we take the ceiling
        int x_end = static_cast<int>(std::ceil(bbox.x_max));
        int y_end = static_cast<int>(std::ceil(bbox.y_max));

        // Clamp to grid boundaries
        x_start = std::max(0, x_start);
        y_start = std::max(0, y_start);
        x_end = std::min(C_, x_end);
        y_end = std::min(R_, y_end);

        for (int y = y_start; y < y_end; ++y) {
            for (int x = x_start; x < x_end; ++x) {
                congestion_map_U_[y][x]++;
            }
        }
    }

    // 3. Compute CC
    const double N = R_ * C_;
    double sum_U = 0.0;
    double sum_U_squared = 0.0;

    for (int y = 0; y < R_; ++y) {
        for (int x = 0; x < C_; ++x) {
            const double u_xy = congestion_map_U_[y][x];
            sum_U += u_xy;
            sum_U_squared += (u_xy * u_xy);
        }
    }

    if (sum_U == 0.0) {
        return 1.0; // No nets, perfect uniformity
    }

    double avg_U = sum_U / N;
    double avg_U_squared_val = avg_U * avg_U;

    if (avg_U_squared_val == 0.0) {
        // Should not happen if sum_U > 0, but as a safeguard
        return 1.0;
    }

    double avg_of_squares = sum_U_squared / N;

    return avg_of_squares / avg_U_squared_val;
}


// --- Output Functions ---

void Placer::write_output(const std::string& output_filename) const {
    // 1. Check if vector is valid
    if (circuit_.blocks.empty()) {
        std::cerr << "Warning: No blocks to write!" << std::endl;
        return;
    }

    std::ofstream file(output_filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open output file: " + output_filename);
    }

    for (size_t i = 0; i < circuit_.blocks.size(); ++i) {
        const auto& block = circuit_.blocks[i];
        if (block.x == -1 || block.y == -1) {
            throw std::runtime_error("Attempting to write output with unplaced block: " + block.name);
        }
        // 2. Bounds Sanity Check
        // If x or y are wild integers (e.g. -2147483648), memory is corrupted.
        if (block.x < 0 || block.x >= C_ || block.y < 0 || block.y >= R_) {
            std::cerr << "CRITICAL ERROR: Block " << i << " has invalid coordinates ("
                      << block.x << ", " << block.y << "). Skipping." << std::endl;
            continue;
        }
        file << block.name << " " << block.x << " " << block.y << "\n";
    }

    file.close();
}

void Placer::print_placement_grid() const {
    std::cerr << "\n--- Placement Grid ---" << std::endl;
    const int cell_width = 5; // Width for each cell in the printout

    // Print header (X coordinates)
    std::cerr << "      "; // Padding for Y-axis label
    for (int x = 0; x < C_; ++x) {
        std::cerr << std::setw(cell_width) << std::left << x;
    }
    std::cerr << std::endl;

    // Print top border
    std::cerr << "     +";
    for (int x = 0; x < C_; ++x) {
        std::cerr << std::string(cell_width - 1, '-') << "+";
    }
    std::cerr << std::endl;

    // Print grid rows
    for (int y = 0; y < R_; ++y) {
        std::cerr << std::setw(4) << std::right << y << " | "; // Y-axis label
        for (int x = 0; x < C_; ++x) {
            std::string name = grid_[y][x];
            if (name.empty()) {
                name = "."; // Empty site
            }
            // Truncate if name is too long
            if (name.length() > cell_width - 2) {
                name = name.substr(0, cell_width - 2);
            }
            std::cerr << std::setw(cell_width - 1) << std::left << name << "|";
        }
        std::cerr << std::endl;
    }
    std::cerr << "--- End of Grid ---" << std::endl;
}