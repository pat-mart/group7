#include <filesystem>
#include <iostream>

#include "include/graph/Graph.h"
#include "include/util/Timer.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main() {

    auto t = Timer();

    std::vector<std::string> file_paths;

    std::cout << "Select a graph: " << std::endl;
    unsigned short index = 0;
    for (const auto& filepath : std::filesystem::directory_iterator("mtx")) {
        std::cout << filepath.path() << " " <<  "(" << index << ")" << std::endl;
        file_paths.push_back(filepath.path());
        index++;
    }

    std::cin >> index;

    t.start();
    Graph graph = Graph::from_mtx(file_paths[index], false, false);
    t.stop();

    std::cout << "Graph construction time elapsed: " << t.elapsed() << std::endl;
    t.reset();

    t.start();
    const std::vector<unsigned long> order = graph.bfs_traversal(2);
    t.stop();

    std::cout << "BFS time elapsed: " << t.elapsed() << " with size " << order.size() << std::endl;
    t.reset();

    t.start();
    auto td_metrics = graph.get_td();
    t.stop();

    auto &td_adj = std::get<0>(td_metrics);
    auto &td_bags = std::get<1>(td_metrics);
    auto root = std::get<2>(td_metrics);

    std::cout << "Tree decomposition time elapsed: " << t.elapsed() << std::endl;
    std::cout << "Treewidth: " << Graph::treewidth(td_bags) << std::endl;

    t.start();
    auto h2h_index = graph.h2h_index();
    t.stop();

    std::cout << "H2H index computation time elapsed: " << t.elapsed() << " seconds" << std::endl;

    for (auto &[bag_id, label] : h2h_index) {
        std::cout << "Bag " << bag_id << ":\n  Ancestor positions: ";
        for (auto p : label.pos) std::cout << p << " ";
        std::cout << "\n  Distances: ";
        for (auto d : label.dis) std::cout << d << " ";
        std::cout << "\n\n";
        
        static int printed = 0;
        if (++printed >= 5) break;
    }
}