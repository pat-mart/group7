//
// Created by Patrick Martin on 12/12/25.
//

#ifndef GROUP7_GRAPH_H
#define GROUP7_GRAPH_H
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <set>

#include "Edge.h"

class Graph {

public:

    using AdjMap = std::unordered_map<unsigned long, std::vector<Edge>>;

    using TreeDecompAdj = std::unordered_map<unsigned long, std::vector<unsigned long>>; // matrix of edges in TD. key denotes bag root v, edge to bag root u
    using TreeDecompBags = std::unordered_map<unsigned long, std::vector<unsigned long>>; // key: bag root v, value: all vertices in X(v) but without weights

    // key v denotes X(v), value psi_i, where 0 <= i < |X(v)|  denotes weight of each vertex in X(v) to v
    // populated after sorting bags by ordering
    using TreeDecompWeights = std::unordered_map<unsigned long, std::vector<unsigned long>>;

    Graph() = default;

    explicit Graph(AdjMap adj);

    static Graph from_mtx(const std::string &path, bool weighted=false, bool directed=false);

    static unsigned long treewidth(TreeDecompBags& bags);

    std::vector<unsigned long> get_star(unsigned long vertex);

    [[nodiscard]] std::vector<unsigned long> bfs_traversal(unsigned long start);

    [[nodiscard]] unsigned long pop_min_degree_vertex();

    void eliminate_vertex(unsigned long v);

    void populate_buckets();

    bool edge_exists(unsigned long u, unsigned long v) const;

    std::vector<unsigned long> get_neighbors(unsigned long vertex);

    unsigned long get_edge_weight(unsigned long u, unsigned long v);

    void add_edge_cache(unsigned long u, unsigned long v);

    void remove_edge_cache(unsigned long u, unsigned long v);

    void change_edge_weight_cache(unsigned long u, unsigned long v);

    // returns adj, bags, root of tree decomposition
    std::tuple<TreeDecompAdj, TreeDecompBags, unsigned long> get_td();

    // h2h index for a vertex
    struct H2HIndex {
        std::vector<unsigned long> pos;
        std::vector<double> dis;
    };

    std::unordered_map<unsigned long, std::vector<unsigned long>> build_ancestor_arrays(const TreeDecompAdj& td_adj, unsigned long root);
    std::unordered_map<unsigned long, H2HIndex> h2h_index();


private:
    AdjMap adj;
    unsigned long num_vertices = 0;

    TreeDecompAdj td_adj;
    TreeDecompBags td_bags;
    TreeDecompWeights td_weights;

    std::unordered_set<uint64_t> edge_set;

    std::vector<std::list<unsigned long>> buckets;
    std::vector<unsigned long> degrees;
    std::vector<std::list<unsigned long>::iterator> bucket_position;
};

#endif //GROUP7_GRAPH_H