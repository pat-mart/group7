//
// Created by Patrick Martin on 12/12/25.
//

#ifndef GROUP7_GRAPH_H
#define GROUP7_GRAPH_H
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <vector>

#include "Edge.h"

class Graph {

public:

    using AdjMap = std::unordered_map<uint32_t, std::vector<Edge>>;

    using TreeDecompAdj = std::vector<std::vector<uint32_t>>; // matrix of edges in TD. key denotes bag root v, edge to bag root u
    using TreeDecompBags = TreeDecompAdj; // key: bag root v, value: all vertices in X(v) but without weights

    using Pos = std::vector<std::vector<uint32_t>>;
    using Dis = std::vector<std::vector<uint32_t>>;

    using TreeDecompBagEdges = std::vector<std::vector<Edge>>;
    using TreeDecompWeights = std::vector<std::vector<uint32_t>>;

    Graph() = default;

    explicit Graph(AdjMap adj, bool populate_buckets);

    static Graph from_mtx(const std::string &path, bool weighted=false, bool directed=false);

    [[nodiscard]] std::vector<uint32_t> bfs_traversal(uint32_t start);
    uint32_t get_num_vertices() const {
        return num_vertices;
    }

    // returns adj, bags, root of tree decomposition
    std::tuple<TreeDecompAdj, TreeDecompBags, uint32_t> get_td();

    std::tuple<Pos, Dis> get_h2h();
    [[nodiscard]] std::vector<uint32_t> get_top_down_ordering() const;
    [[nodiscard]] std::vector<uint32_t> get_bag_path(uint32_t v) const;
    [[nodiscard]] uint32_t h2h_query(uint32_t u, uint32_t v);
    uint32_t get_h2h_size();
    size_t get_treeheight();
    size_t get_treeheight_r(uint32_t root, uint32_t pred, size_t depth);

    void populate_buckets();
    void populate_buckets_min_fill();
    void clear_buckets();
    uint32_t get_fill(uint32_t u);

    [[nodiscard]] bool edge_exists(uint32_t u, uint32_t v) const;
    [[nodiscard]] uint32_t pop_next_vertex();
    [[nodiscard]] std::vector<uint32_t> get_star(uint32_t vertex) const;
    void eliminate_vertex(uint32_t v, bool is_min_degree);
    static uint32_t treewidth(TreeDecompBags& bags);

    [[nodiscard]] std::vector<uint32_t> get_neighbors(uint32_t vertex) const;
    [[nodiscard]] uint32_t get_edge_weight(uint32_t u, uint32_t v) const;

    void add_edge_cache(uint32_t u, uint32_t v);
    void remove_edge_cache(uint32_t u, uint32_t v);

    [[nodiscard]] std::vector<uint32_t> get_random_ordering() const;
    uint32_t num_vertices = 0;
    uint32_t num_edges = 0;

private:
    AdjMap adj{};
    
    TreeDecompAdj td_adj;
    TreeDecompBags td_bags;

    TreeDecompBagEdges td_bag_edges;
    TreeDecompWeights td_weights;

    std::vector<uint32_t> parent_map;

    uint32_t td_root = 1e9;
    size_t treeheight;

    float avg_degree;
    std::vector<std::vector<uint32_t>> vertex_betweenness;

    std::tuple<Pos, Dis> h2h;

    std::unordered_set<uint64_t> edge_set;

    std::vector<std::list<uint32_t>> buckets;
    std::vector<uint32_t> heuristic_vals;
    std::vector<std::list<uint32_t>::iterator> bucket_position;

    static uint32_t index_of(const std::vector<uint32_t>&, uint32_t v);

    uint32_t lca(uint32_t u, uint32_t v);
    void update_bucket(uint32_t u, uint32_t heuristic_val);
};

#endif //GROUP7_GRAPH_H
