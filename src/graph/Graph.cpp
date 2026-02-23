//
// Created by Patrick Martin on 12/12/25.
//

#include <utility>
#include <fstream>
#include <sstream>
#include <ranges>
#include <list>
#include <unordered_set>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cassert>
#include <utility>
#include <algorithm>

#include "graph/Graph.h"
#include "util/Oracle.h"

#include <iostream>
#include <random>

Graph::Graph(AdjMap adj, bool populate_buckets) : adj(std::move(adj)) {
    if(populate_buckets) {
        buckets.resize(10000);

        heuristic_vals.resize(this->adj.size());
        bucket_position.resize(this->adj.size());
    }
}

// This method generated with Claude Sonnet 4.5
Graph Graph::from_mtx(const std::string &path, bool weighted, bool directed) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Could not open file " + path);
    }

    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line[0] != '%') {
            break;
        }
    }

    if (line.empty()) {
        throw std::runtime_error("No matrix header found in file " + path);
    }

    std::istringstream header(line);

    int n, m, nnz;
    header >> n >> m >> nnz;

    AdjMap adj;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '%') continue;
        std::istringstream iss(line);

        uint32_t u, v;
        uint32_t w = 1;
        iss >> u >> v;
        if (weighted)
            iss >> w;

        u--; v--;

        adj[u].push_back({v, w});
        if (!directed && u != v) {
            adj[v].push_back({u, w});
        }
    }

    auto g = Graph(adj, false);

    return g;
}

std::vector<uint32_t> Graph::bfs_traversal(const uint32_t start) {
    std::vector<uint32_t> order;
    std::vector visited(adj.size(), false);

    std::queue<uint32_t> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        uint32_t const u = q.front();
        q.pop();
        order.push_back(u);
        for (const auto&[to, w] : adj.at(u)) {
            if (!visited[to]) {
                visited[to] = true;
                q.push(to);
            }
        }

        num_vertices += 1;
    }

    return order;
}

std::vector<uint32_t> Graph::get_neighbors(const uint32_t vertex) const {
    std::vector<uint32_t> neighbors;
    for (const auto &[v, w] : adj.at(vertex)) {
        neighbors.push_back(v);
    }

    return neighbors;
}

std::vector<uint32_t> Graph::get_star(const uint32_t vertex) const {
    auto star = std::vector<uint32_t>();
    star.push_back(vertex);

    for (const auto &[to, w]: adj.at(vertex)) {
        star.push_back(to);
    }

    return star;
}

void Graph::populate_buckets() {
    for (const auto &[u, edges] : adj) {
        uint32_t deg = edges.size();

        buckets[deg].push_back(u);
        bucket_position[u] = std::prev(buckets[deg].end());
        heuristic_vals[u] = deg;

        // for (const auto&[to, w] : edges) {
        //     add_edge_cache(u, to);
        // }
    }
}

void Graph::populate_buckets_min_fill() {
    for (const auto &[u, edges] : adj) {
        for (const auto& [to, w] : edges) {
            add_edge_cache(u, to);
        }
    }

    for (const auto &[u, edges] : adj) {
        uint32_t fill_num = get_fill(u);

        buckets[fill_num].push_back(u);
        bucket_position[u] = std::prev(buckets[fill_num].end());
        heuristic_vals[u] = fill_num;
    }
}

void Graph::clear_buckets() {
    buckets.clear();
    
    heuristic_vals.clear();
}

// fill := number of 'missing edges' between neighbors to make a clique 
uint32_t Graph::get_fill(uint32_t v) {
    uint32_t fill = 0;
    const auto& neighbors = get_neighbors(v);

    for(size_t i = 0; i < neighbors.size(); i++) {
        for(size_t j = i + 1; j < neighbors.size(); j++) {
            uint32_t u = neighbors[i];
            uint32_t w = neighbors[j];

            if (!edge_exists(u, w)) {
                fill++;
            }
        }
    }

    return fill;
}

void Graph::eliminate_vertex(const uint32_t v, bool is_min_degree) {
    const auto& neighbors = get_neighbors(v);

    // fills in edges w/ updated weights
    for (size_t i = 0; i < neighbors.size(); i++) {
        for (size_t j = i + 1; j < neighbors.size(); j++) {
            uint32_t u = neighbors[i];
            uint32_t w = neighbors[j];

            const uint32_t uvw_weight = get_edge_weight(u, v) + get_edge_weight(v, w);

            auto& adj_w = adj.at(w);
            auto& adj_u = adj.at(u);

            if (!edge_exists(u, w)) {
                adj_u.push_back({w, uvw_weight});
                adj_w.push_back({u, uvw_weight});

                add_edge_cache(u, w);
                add_edge_cache(w, u);

            } else if (uvw_weight < get_edge_weight(u, w)) {
                // FIXME ?
                adj_u.erase(std::ranges::find_if((adj)[u], [&](const Edge& e) {return e.to == w;}));
                adj_w.erase(std::ranges::find_if(adj_w, [&](const Edge& e) {return e.to == u;}));

                adj_u.push_back({w, uvw_weight});
                adj_w.push_back({u, uvw_weight});
            }
        }
    }

    // add edge weights to td_weights
    for (const auto& neighbor: neighbors) {
        const auto w = get_edge_weight(neighbor, v);
        td_bag_edges[v].push_back({neighbor, w});
    }
    td_bag_edges[v].push_back({v, 0});


    // removes any outward edges from neighbors to vertex
    for (const auto& neighbor : neighbors) {
        remove_edge_cache(neighbor, v);
        remove_edge_cache(v, neighbor);
        adj.at(neighbor).erase(std::ranges::find_if(adj.at(neighbor), [&](const Edge& e) {return e.to == v;}));
    }

    // erases vertex
    adj.erase(v);
    num_vertices -= 1;

    // updates bucket value of neighbors after edge fill-in
    for (uint32_t neighbor : neighbors) {
        const uint32_t d1 = heuristic_vals[neighbor];

        const uint32_t d2 = get_fill(neighbor);

        buckets[d1].erase(bucket_position[neighbor]);
        buckets[d2].push_front(neighbor);
        bucket_position[neighbor] = buckets[d2].begin();
        heuristic_vals[neighbor] = d2;
    }
}

void Graph::update_bucket(const uint32_t u, const uint32_t heuristic_val) {
    const uint32_t old_bucket = heuristic_vals[u];
    const uint32_t new_bucket = heuristic_val;

    buckets[old_bucket].erase(bucket_position[old_bucket]);
    buckets[new_bucket].push_front(u);
    bucket_position[u] = buckets[new_bucket].begin();
    heuristic_vals[u] = new_bucket;
}

// obtains vertex with minimum degree and pops it from its corresponding bucket
uint32_t Graph::pop_next_vertex() {
    uint32_t min_bucket = 0;

    while (buckets[min_bucket].empty()) {
        min_bucket++;
    }

    const uint32_t v = buckets[min_bucket].front();
    buckets[min_bucket].pop_front();

    return v;
}

bool Graph::edge_exists(const uint32_t u, const uint32_t v) const {
    const uint64_t edge = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
    return edge_set.contains(edge);
}

uint32_t Graph::get_edge_weight(const uint32_t u, const uint32_t v) const {
    if (u == v) return 0;

    for (const Edge& e : adj.at(u)) {
        if (e.to == v) return e.w;
    }

    return UINT32_MAX;
}

void Graph::add_edge_cache(const uint32_t u, const uint32_t v) {
    // first 32 bits encode u (first shifted left 32 bits), last 32 bits encode v
    const uint64_t edge_cache = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);

    edge_set.insert(edge_cache);
}

void Graph::remove_edge_cache(const uint32_t u, const uint32_t v) {
    const uint64_t edge_cache = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);

    edge_set.erase(edge_cache);
}

// DNF in reasonable amount of time even on very small graph
std::vector<uint32_t> Graph::get_random_ordering() const {
    std::vector<uint32_t> ordering(adj.size());
    for (uint32_t u = 0; u < adj.size(); u++) {
        ordering[u] = u;
    }

    std::mt19937_64 rng(std::random_device{}());
    std::ranges::shuffle(ordering, rng);

    return ordering;
}

std::tuple<Graph::TreeDecompAdj, Graph::TreeDecompBags, uint32_t> Graph::get_td() {
    Graph h = Graph(adj, true);

    const auto adj_size = adj.size();

    h.num_vertices = adj_size;
    // h.populate_buckets();
    h.populate_buckets_min_fill();
    h.td_bag_edges.resize(adj_size);

    std::vector<uint32_t> ordering(adj_size);
    parent_map.resize(adj_size);

    td_bag_edges.resize(adj_size);
    td_weights.resize(adj_size);

    td_bags.clear();
    td_adj.clear();

    td_adj.resize(adj_size);
    td_bags.resize(adj_size);

    for (size_t i = 0; i < adj.size(); i++) {
        uint32_t v = h.pop_next_vertex();

        td_bags[v] = h.get_star(v);
        h.eliminate_vertex(v, true);

        ordering[v] = i;

        if (i % static_cast<int>(num_vertices / 10) == 0) {
            std::cout << "Eliminated vertex " << i << " " << 10 * i / static_cast<int>(num_vertices / 10) << "%" << std::endl;
        }
    }

    for (uint32_t v : adj | std::views::keys) {
        const auto& bag = td_bags.at(v);

        uint32_t min_u = v;
        uint32_t best = UINT32_MAX;

        for (const uint32_t u : bag) {
            if (u != v && ordering[u] < best) {
                best = ordering[u];
                min_u = u;
            }
        }

        if (best == UINT32_MAX) {
            td_root = v;
            parent_map[v] = v;
            continue;
        }

        td_adj[v].push_back(min_u);
        td_adj[min_u].push_back(v);

        parent_map[v] = min_u;
    }

    for (size_t v = 0; v < td_bags.size(); v++) {

        auto& bag = td_bags[v];

        std::ranges::sort(bag, [&](const uint32_t a, const uint32_t b) {return ordering[a] > ordering[b];});

        if (v == td_root) {
            td_weights[v].push_back(0);
            continue;
        }

        const auto& edges = h.td_bag_edges.at(v);

        for (const uint32_t u : bag) {
            
            for (const auto&[to, w] : edges) {
                if (to == u && u != v) {
                    td_weights[v].push_back(w);
                }
            }
        }

        td_weights[v].push_back(0);
    }

    return {td_adj, td_bags, td_root};
}

std::vector<uint32_t> Graph::get_top_down_ordering() const {
    std::vector<uint32_t> ordering;

    std::queue<uint32_t> q;
    std::unordered_set<uint32_t> visited;

    q.push(td_root);
    visited.insert(td_root);

    while (!q.empty()) {
        uint32_t u = q.front();
        q.pop();
        ordering.push_back(u);

        for (const uint32_t neighbor : td_adj.at(u)) {
            if (!visited.contains(neighbor)) {
                q.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }

    return ordering;
}

std::vector<uint32_t> Graph::get_bag_path(const uint32_t v) const {
    std::vector<uint32_t> path;

    if (v == td_root) {
        path.push_back(v);
        return path;
    }

    uint32_t current = v;
    while (current != td_root) {
        path.push_back(current);
        
        uint32_t parent = parent_map[current];
        current = parent;
    }
    
    path.push_back(td_root);
    std::ranges::reverse(path);
    
    return path;
}

uint32_t Graph::lca(const uint32_t u, const uint32_t v) {
    const auto& u_anc = get_bag_path(u);
    const auto& v_anc = get_bag_path(v);

    const auto min_len = std::min(u_anc.size(), v_anc.size());
    uint32_t lca = UINT32_MAX;

    for (size_t i = 0; i < min_len; i++) {
        if (u_anc[i] == v_anc[i]) {
            lca = u_anc[i];
        }
        else {
            break;
        }
    }

    if (lca == UINT32_MAX) {
        throw std::invalid_argument("Graph::lca() failed");
    }

    return lca;
}

uint32_t Graph::h2h_query(const uint32_t u, const uint32_t v) {

    const auto x = lca(u, v);

    uint32_t d = 1e9;
    const auto& dis_map = std::get<1>(h2h);
    const auto& pos_map = std::get<0>(h2h);

    for (const uint32_t i : pos_map.at(x)) {
        d = std::min(d, dis_map.at(u)[i] + dis_map.at(v)[i]);
    }

    return d;
}

uint32_t Graph::index_of(const std::vector<uint32_t>& b, const uint32_t v) {
    for (uint32_t i = 0; i < b.size(); ++i) {
        if (b[i] == v) {return i;}
    }

    throw std::out_of_range("Vertex not found");
}

std::tuple<Graph::Pos, Graph::Dis> Graph::get_h2h() {
    Pos pos(td_bags.size());
    Dis dis(td_bags.size());

    const std::vector<uint32_t> ordering = get_top_down_ordering();

    for (const uint32_t v_bag : ordering) {
        const auto& anc = get_bag_path(v_bag);
        auto& bag = td_bags.at(v_bag); 

        treeheight = std::max(anc.size(), treeheight) - 1;
    
        for (const uint32_t bag_vertex : bag) {
            auto bag_pos_i = index_of(anc, bag_vertex);
            pos[v_bag].push_back(bag_pos_i);
        }
        std::ranges::sort(pos[v_bag]);

        for (uint32_t i = 0; i < anc.size()-1; i++) {

            dis[v_bag].push_back(1e9);

            for (uint32_t j = 0; j < bag.size()-1; j++) {
                uint32_t d;
                uint32_t xj_vertex = bag.at(j);

                if (pos[v_bag][j] > i) {
                    d = dis[xj_vertex][i];
                } else {
                    uint32_t v_bag_anc_i = anc[i];
                    const uint32_t dis_index = pos[v_bag][j];

                    d = dis[v_bag_anc_i][dis_index];
                }

                dis[v_bag][i] = std::min(dis[v_bag][i], td_weights.at(v_bag)[j] + d);
            }
        }

       dis[v_bag].push_back(0);
    }

    h2h = {std::move(pos), std::move(dis)};

    return h2h;
}

// Ask Hector about this -- huge discrepancy between index sizes in RL paper that make me skeptical of results 
uint32_t Graph::get_h2h_size() {
    const auto& pos = std::get<0>(h2h);
    const auto& dis = std::get<1>(h2h);

    unsigned long pos_sum = sizeof(pos);
    unsigned long dis_sum = sizeof(dis);

    for (const std::vector<uint32_t>& pos_arr : pos) {
        pos_sum += sizeof(pos_arr);
    }

    for (const std::vector<uint32_t>& dis_arr : dis) {
        dis_sum += sizeof(dis_arr);
    }

    return pos_sum + dis_sum;
}


uint32_t Graph::treewidth(TreeDecompBags& bags) {
    uint32_t tw = 0;

    for (auto &bag: bags) {
        if (bag.size() > tw) {
            tw = bag.size();
        }
    }

    return tw - 1;
}

size_t Graph::get_treeheight() {
    return get_treeheight_r(td_root, -1, 0);
}

size_t Graph::get_treeheight_r(uint32_t root, uint32_t pred, size_t depth) {
    const auto& neighbors = td_adj[root];

    if (neighbors.size() == 1 && pred != -1) { // leaf
        return depth;
    }

    const size_t c_depth = depth;

    // height of tree rooted at v_root: current depth + maximum height of all of its children
    for (const uint32_t neighbor : neighbors) {
        if(neighbor != pred) {
            depth = std::max<size_t>(depth, get_treeheight_r(neighbor, root, c_depth+1));
        }
    }

    return depth;
}
