#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include "graph/Graph.h"
#include "util/Timer.h"
#include "util/Oracle.h"

int main() {

    auto t = Timer();

    std::vector<std::string> file_paths = {
         "mtx/road-minnesota.mtx",
         "mtx/road-usroads-48.mtx",
         "mtx/road-belgium-osm.mtx",
         "mtx/road-germany-osm.mtx",
         "mtx/road-great-britain-osm.mtx",
         "mtx/road-italy-osm.edges",
         "mtx/road-netherlands-osm.mtx",
         "mtx/road-roadNET-PA.mtx",
    };

    auto now = std::chrono::system_clock::now();
    std::time_t lt = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&lt);

    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << tm.tm_mon + 1
        << std::setw(2) << tm.tm_mday
        << std::setw(2) << tm.tm_hour
        << std::setw(2) << tm.tm_min;

    std::string filename = "./out/" + oss.str() + ".csv";
    std::ofstream file(filename);

    file << "graph,heuristic,treewidth,treeheight,td_time_s,h2h_time_s,h2h_size_bytes,avg_degree,N,E\n";

    // helper: BFS reachability count from a root in td_adj
    auto count_reachable = [](const std::vector<std::vector<uint32_t>>& adj, uint32_t root) -> size_t {
        std::queue<uint32_t> q;
        std::unordered_set<uint32_t> seen;
        q.push(root);
        seen.insert(root);
        while (!q.empty()) {
            uint32_t node = q.front(); q.pop();
            for (uint32_t nb : adj.at(node)) {
                if (!seen.count(nb)) {
                    seen.insert(nb);
                    q.push(nb);
                }
            }
        }
        return seen.size();
    };

    // ── Define which heuristic runs to execute ────────────────────────────────
    struct HeuristicRun {
        Graph::Heuristic heuristic;
        std::string label;
        float alpha;
    };

    const std::vector<HeuristicRun> fixed_runs = {
        //{ Graph::Heuristic::MIN_DEGREE,          "min-degree",          0.0f },
        { Graph::Heuristic::MIN_NEIGHBOR_DEGREE, "min-neighbor-degree", 0.0f },
        // { Graph::Heuristic::MIN_FILL,         "min-fill",            0.0f },
        // { Graph::Heuristic::LEX_BFS,          "lex-bfs",             0.0f },
    };
    // ─────────────────────────────────────────────────────────────────────────

    const bool run_h2h = false;

    for (const std::string& path : file_paths) {
        std::string name = std::filesystem::path(path).stem().string();
        std::cout << "\n=== " << name << " ===" << std::endl;

        // --- Fixed heuristic runs ---
        for (const auto& run : fixed_runs) {
            Graph graph = Graph::from_mtx(path, false, false);

            std::cout << run.label << "..." << std::flush;

            t.reset(); t.start();
            auto td_metrics = graph.get_td(run.heuristic, run.alpha);
            t.stop();
            double td_time = t.elapsed();

            auto& td_bags = std::get<1>(td_metrics);
            uint32_t tw = Graph::treewidth(td_bags);
            size_t th = graph.get_treeheight();
            std::cout << " tw=" << tw << " th=" << th << std::endl;

            // ── debug ──────────────────────────────────────────────────────
            std::cout << "  td_root=" << graph.td_root << std::endl;
            std::cout << "  root_children=" << graph.td_adj.at(graph.td_root).size() << std::endl;
            std::cout << "  reachable_from_root=" << count_reachable(graph.td_adj, graph.td_root)
                      << " (total nodes=" << graph.num_vertices << ")" << std::endl;
            // ──────────────────────────────────────────────────────────────

            double h2h_time = 0.0;
            uint32_t h2h_size = 0;
            if (run_h2h) {
                t.reset(); t.start();
                graph.get_h2h();
                t.stop();
                h2h_time = t.elapsed();
                h2h_size = graph.get_h2h_size();
            }

            file << name << ","
                 << run.label << ","
                 << tw << ","
                 << th << ","
                 << td_time << ","
                 << h2h_time << ","
                 << h2h_size << ","
                 << graph.get_avg_degree() << ","
                 << graph.num_vertices << ","
                 << graph.get_num_edges() << "\n";

            std::cout << "  td=" << td_time << "s"
                      << (run_h2h ? " h2h=" + std::to_string(h2h_time) + "s" : " [h2h skipped]")
                      << std::endl;
        }

        // --- Hybrid alpha sweep ---
        const std::unordered_map<std::string, std::vector<float>> alpha_map = {
            // {"road-minnesota",            {0.6f, 0.8f}},
            // {"road-usroads-48",        {0.4f, 0.6f, 0.8f}},
            // {"road-belgium-osm",       {0.6f, 0.8f}},
            // {"road-germany-osm",       {0.6f, 0.8f}},
            // {"road-great-britain-osm", {0.6f, 0.8f}},
            // {"road-italy-osm",         {0.6f, 0.8f}},
            // {"road-netherlands-osm",   {0.6f, 0.8f}},
            // {"road-roadNET-PA",        {0.6f, 0.8f}},
        };
        const std::vector<float> alphas = alpha_map.count(name)
            ? alpha_map.at(name)
            : std::vector<float>{};

        for (float alpha : alphas) {
            Graph graph = Graph::from_mtx(path, false, false);

            std::ostringstream label;
            label << "hybrid-" << static_cast<int>(alpha * 10);
            std::string heuristic_label = label.str();

            std::cout << heuristic_label << "..." << std::flush;

            t.reset(); t.start();
            auto td_metrics = graph.get_td(Graph::Heuristic::HYBRID, alpha);
            t.stop();
            double td_time = t.elapsed();

            auto& td_bags = std::get<1>(td_metrics);
            uint32_t tw = Graph::treewidth(td_bags);
            size_t th = graph.get_treeheight();
            std::cout << " tw=" << tw << " th=" << th << std::endl;

            // ── debug ──────────────────────────────────────────────────────
            std::cout << "  td_root=" << graph.td_root << std::endl;
            std::cout << "  root_children=" << graph.td_adj.at(graph.td_root).size() << std::endl;
            std::cout << "  reachable_from_root=" << count_reachable(graph.td_adj, graph.td_root)
                      << " (total nodes=" << graph.num_vertices << ")" << std::endl;
            // ──────────────────────────────────────────────────────────────

            double h2h_time = 0.0;
            uint32_t h2h_size = 0;
            if (run_h2h) {
                t.reset(); t.start();
                graph.get_h2h();
                t.stop();
                h2h_time = t.elapsed();
                h2h_size = graph.get_h2h_size();
            }

            file << name << ","
                 << heuristic_label << ","
                 << tw << ","
                 << th << ","
                 << td_time << ","
                 << h2h_time << ","
                 << h2h_size << ","
                 << graph.get_avg_degree() << ","
                 << graph.num_vertices << ","
                 << graph.get_num_edges() << "\n";

            std::cout << "  td=" << td_time << "s"
                      << (run_h2h ? " h2h=" + std::to_string(h2h_time) + "s" : " [h2h skipped]")
                      << std::endl;
        }
    }

    file.close();
    std::cout << "\nResults written to " << filename << std::endl;

    return 0;
}