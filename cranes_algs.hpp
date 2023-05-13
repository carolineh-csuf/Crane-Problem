///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes {

// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    // Compute maximum path length, and check that it is legal.
    const size_t max_steps = setting.rows() + setting.columns() - 2;

    std::cout << "max_steps:" << max_steps << " ";

    assert(max_steps < 64);

    path best(setting);

    for (size_t steps = 0; steps <= max_steps; steps++) {
        for (int i = 0; i < (1 << steps); i++) {
            size_t max_cranes = best.total_cranes();
            path valid_path(setting);

            for (size_t j = 0; j <= steps - 1; j++){
                step_direction step_direction;

                if (((i >> j) & 1) == 0){
                    step_direction = STEP_DIRECTION_SOUTH;
                } else {
                    step_direction = STEP_DIRECTION_EAST;
                }

                if (valid_path.is_step_valid(step_direction)){
                    valid_path.add_step(step_direction);
                } else {
                    break;
                }
            }

            if (valid_path.total_cranes() > max_cranes){
                max_cranes = valid_path.total_cranes();
                best = valid_path;
            }
        }
    }
    return best;

}

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
//path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    using cell_type = std::optional<path>;

    std::vector<std::vector<cell_type> > A(setting.rows(),
                                           std::vector<cell_type>(setting.columns()));

    A[0][0] = path(setting);
    assert(A[0][0].has_value());

    for (coordinate r = 0; r < setting.rows(); ++r) {
        for (coordinate c = 0; c < setting.columns(); ++c) {

            if (setting.get(r, c) == CELL_BUILDING){
                A[r][c].reset();
                continue;
            }

            cell_type from_above = std::nullopt;
            cell_type from_left = std::nullopt;

            bool is_from_above = false;
            bool is_from_left = false;

            //The from-above path only exists when we are not on the top row (r>0)
            if (r>0 && A[r-1][c].has_value()){
                is_from_above = true;
            }

            //The from-left path only exists when we are not on the leftmost column (c>0)
            if (c>0 && A[r][c-1].has_value()){
                is_from_left = true;
            }

            if (is_from_above && is_from_left){
                if  ( A[r-1][c]->total_cranes () > A[r][c-1]-> total_cranes ()) {
                    if( A[r-1][c]->is_step_valid(STEP_DIRECTION_SOUTH)){
                        from_above = A[r-1][c];
                        from_above->add_step(STEP_DIRECTION_SOUTH);
                        A[r][c] = from_above;
                    }
                } else if(A[r][c-1]->is_step_valid(STEP_DIRECTION_EAST)){
                    from_left = A[r][c-1];
                    from_left->add_step(STEP_DIRECTION_EAST);
                    A[r][c] = from_left;
                }
            }
            else if(is_from_left) {
                if(A[r][c-1]->is_step_valid(STEP_DIRECTION_EAST)){
                    from_left = A[r][c-1];
                    from_left->add_step(STEP_DIRECTION_EAST);
                    A[r][c] = from_left;
                }
            }
            else if(is_from_above) {
                if( A[r-1][c]->is_step_valid(STEP_DIRECTION_SOUTH)){
                    from_above = A[r-1][c];
                    from_above->add_step(STEP_DIRECTION_SOUTH);
                    A[r][c] = from_above;
                }
            }
        }
    }

    //post-processing step to find the path reaching most cranes
    cell_type* best = &(A[0][0]);
    for (coordinate r = 0; r < setting.rows(); ++r) {
        for (coordinate c = 0; c < setting.columns(); ++c) {
            if (A[r][c].has_value()){
                if(A[r][c]->total_cranes() > (*best)->total_cranes()) {
                    best = &(A[r][c]);
                }
            }
        }
    }

    assert(best->has_value());
    // std::cout << "total cranes" << (**best).total_cranes() << std::endl;

    return **best;
}

}
