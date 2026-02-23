#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
#include <ctime>

#include "graph/Graph.h"
#include "util/Timer.h"
#include "util/Oracle.h"

int main() {

    auto t = Timer();

    std::vector<std::string> file_paths;

    for (const auto& filepath : std::filesystem::directory_iterator("mtx")) {
        file_paths.push_back(filepath.path());
    }

    auto now = std::chrono::system_clock::now();
    std::time_t lt = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&lt);


    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << tm.tm_mon + 1   // MM
        << std::setw(2) << tm.tm_mday      // DD
        << std::setw(2) << tm.tm_hour      // HH
        << std::setw(2) << tm.tm_min;      // MM

    std::string filename = "./out/" + oss.str() + ".txt";

    std::ofstream file(filename);
    file << "Start\n";

    for (const std::string& filename : file_paths) {
        t.reset();
        t.start();
        Graph graph = Graph::from_mtx(filename, false, false);    
        t.stop();

        file << "Graph " << filename << " construction time elapsed: " << t.elapsed() << std::endl;
        t.reset();

        t.start();
        const std::vector<uint32_t> order = graph.bfs_traversal(6);
        t.stop();

        std::cout << "BFS time elapsed: " << t.elapsed() << " with size " << order.size() << std::endl;
        file << "BFS time elapsed: " << t.elapsed() << " with size " << order.size() << std::endl;
        t.reset();

        t.start();
        auto td_metrics = graph.get_td();
        t.stop();

        auto& td_bags = std::get<1>(td_metrics);

        file << "Tree decomposition time elapsed: " << t.elapsed() << std::endl;
        file << "Treewidth: " << Graph::treewidth(td_bags) << std::endl;
        t.reset();

        // t.start();
        // const auto& pos_dis = graph.get_h2h();
        // t.stop();

        // file << "H2H construction time elapsed: " << t.elapsed() << std::endl;
        file << "Treeheight: " << graph.get_treeheight() << std::endl;

        // if (GraphUtil::verify_h2h(graph, graph.get_num_vertices(), 50)) {
        //     file << "Valid H2H across 50 samples" << std::endl;
        //     file << "H2H size: " << graph.get_h2h_size() << std::endl;
        // }

        file << "End\n" << std::endl;
    }    
    
    file.close();

    return 0;
}