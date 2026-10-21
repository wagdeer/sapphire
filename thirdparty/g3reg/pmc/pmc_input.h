/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#ifndef PMC_INPUT_H_
#define PMC_INPUT_H_

#include "pmc_headers.h"
#include "pmc_utils.h"

using namespace std;

namespace pmc {
class input {
    public:
        // instance variables
        int algorithm;
        int threads;
        int experiment;
        int lb;
        int ub;
        int param_ub;
        int adj_limit;
        double time_limit;
        double remove_time;
        bool graph_stats;
        bool verbose;
        bool help;
        bool MCE;
        bool decreasing_order;
        string heu_strat;
        string format;
        string graph;
        string output;
        string edge_sorter;
        string vertex_search_order;

        input() {
            // default values
            algorithm = 0;
            threads = 1;
            experiment = 0;
            lb = 0;
            ub = 0;
            param_ub = 0;
            adj_limit = 20000;
            time_limit = 60 * 60; 			// max time to search
            remove_time = 4.0; 				// time to wait before reducing graph
            verbose = false;
            graph_stats = false;
            help = false;
            MCE = false;
            decreasing_order = false;
            heu_strat = "kcore";
            vertex_search_order = "deg";
            format = "mtx";
            graph = "data/sample.mtx";
            output = "";
            string edge_sorter = "";

            // both off, use default alg
            if (heu_strat == "0" && algorithm == -1)
                algorithm = 0;

            if (threads <= 0) threads = 1;
        }



};
} // namespace pmc
#endif
