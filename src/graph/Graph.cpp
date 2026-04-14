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
#include <algorithm>
#include <utility>
#include <algorithm>

#include "graph/Graph.h"
#include "util/Oracle.h"

#include <iostream>
#include <random>

Graph::Graph(AdjMap adj, bool populate_buckets) : adj(std::move(adj)) {
    if(populate_buckets) {
        buckets.resize(100000);

        uint32_t max_id = 0;
        for (const auto& [u, _] : this->adj) {
            if (u > max_id) max_id = u;
        }
        heuristic_vals.resize(max_id + 1);
        bucket_position.resize(max_id + 1);
    }
}

// This method generated with Claude Sonnet 4.5
Graph Graph::from_mtx(const std::string &path, bool weighted, bool directed) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Could not open file " + path);
    }

    // detect format by extension
    bool is_edges = path.size() >= 6 && path.substr(path.size() - 6) == ".edges";

    std::string line;
    AdjMap adj;

    if (is_edges) {
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#' || line[0] == '%') continue;
            std::istringstream iss(line);
            uint32_t u, v;
            if (!(iss >> u >> v)) continue;
            u--; v--;
            adj[u].push_back({v, 1});
            if (!directed && u != v)
                adj[v].push_back({u, 1});
        }
    } else {
        while (std::getline(in, line)) {
            if (!line.empty() && line[0] != '%') break;
        }

        if (line.empty())
            throw std::runtime_error("No matrix header found in file " + path);

        std::istringstream header(line);
        int n, m, nnz;
        header >> n >> m >> nnz;

        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '%') continue;
            std::istringstream iss(line);

            uint32_t u, v;
            uint32_t w = 1;
            iss >> u >> v;
            if (weighted) iss >> w;

            u--; v--;

            adj[u].push_back({v, w});
            if (!directed && u != v)
                adj[v].push_back({u, w});
        }
    }

    auto g = Graph(adj, false);

    uint32_t edge_count = 0;
    uint32_t max_id = 0;
    for (const auto& [u, edges] : adj) {
        edge_count += edges.size();
        if (u > max_id) max_id = u;
    }
    g.num_edges = directed ? edge_count : edge_count / 2;
    g.num_vertices = adj.size();
    g.max_vertex_id = max_id;
    g.weighted_graph = weighted;

    return g;
}

std::vector<uint32_t> Graph::bfs_traversal(const uint32_t start) {
    std::vector<uint32_t> order;
    std::vector visited(max_vertex_id + 1, false);

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
    }
    min_bucket_hint = 0;
}

void Graph::populate_buckets_min_fill() {
    for (const auto &[u, edges] : adj) {
        for (const auto& [to, w] : edges) {
            add_edge_cache(u, to, w);
        }
    }

    for (const auto &[u, edges] : adj) {
        uint32_t fill_num = get_fill(u);

        buckets[fill_num].push_back(u);
        bucket_position[u] = std::prev(buckets[fill_num].end());
        heuristic_vals[u] = fill_num;
    }
    min_bucket_hint = 0;
}

// ── MIN NEIGHBOR DEGREE ──────────────────────────────────────────────────────
//
// Score = sum of current degrees of all alive neighbors.
// Prefer vertices sitting in structurally simpler (low-degree) neighborhoods.
//
// Update strategy: 1-hop only (direct neighbors of eliminated vertex).
// 2-hop neighbors have slightly stale scores but the effect is small on sparse
// road networks (avg degree ~3). This keeps per-step update cost at O(d)
// instead of O(d^2), which is necessary for million-node graphs.

uint32_t Graph::get_neighbor_degree_score(uint32_t v) {
    uint32_t score = 0;
    for (const auto& [u, w] : adj.at(v)) {
        score += static_cast<uint32_t>(adj.at(u).size());
    }
    return score;
}

void Graph::populate_buckets_min_neighbor_degree() {
    for (const auto &[u, edges] : adj) {
        uint32_t score = get_neighbor_degree_score(u);

        // scores grow as fill edges are added during elimination — resize if needed
        if (score >= buckets.size()) buckets.resize(score + 1);

        buckets[score].push_back(u);
        bucket_position[u] = std::prev(buckets[score].end());
        heuristic_vals[u] = score;
    }
    min_bucket_hint = 0;
}

// ─────────────────────────────────────────────────────────────────────────────

std::vector<uint32_t> Graph::get_hybrid_order(float alpha) const {
    auto* self = const_cast<Graph*>(this);

    std::vector<std::pair<uint32_t, uint32_t>> deg_fill;
    deg_fill.reserve(adj.size());

    float max_deg  = 1.0f;
    float max_fill = 1.0f;

    for (const auto& [u, edges] : adj) {
        uint32_t deg  = edges.size();
        uint32_t fill = self->get_fill(u);
        max_deg  = std::max(max_deg,  (float)deg);
        max_fill = std::max(max_fill, (float)fill);
        deg_fill.push_back({deg, fill});
    }

    std::vector<std::pair<float, uint32_t>> scores;
    scores.reserve(adj.size());

    size_t idx = 0;
    for (const auto& [u, edges] : adj) {
        float norm_deg  = deg_fill[idx].first  / max_deg;
        float norm_fill = deg_fill[idx].second / max_fill;
        float score = alpha * norm_deg + (1.0f - alpha) * norm_fill;
        scores.push_back({score, u});
        idx++;
    }

    std::sort(scores.begin(), scores.end());

    std::vector<uint32_t> order;
    order.reserve(scores.size());
    for (const auto& [score, v] : scores) {
        order.push_back(v);
    }
    return order;
}

void Graph::populate_buckets_hybrid(float alpha) {
    hybrid_alpha = alpha;
    for (const auto &[u, edges] : adj) {
        for (const auto& [to, w] : edges) {
            add_edge_cache(u, to, w);
        }
    }

    const bool use_fill = (adj.size() <= 200000);

    if (use_fill) {
        float max_deg = 1.0f, max_fill = 1.0f;
        std::vector<uint32_t> fills;
        fills.reserve(adj.size());
        for (const auto& [u, edges] : adj) {
            max_deg = std::max(max_deg, (float)edges.size());
            uint32_t f = get_fill(u);
            max_fill = std::max(max_fill, (float)f);
            fills.push_back(f);
        }
        std::vector<std::pair<float, uint32_t>> to_insert;
        to_insert.reserve(adj.size());
        size_t idx = 0;
        for (const auto& [u, edges] : adj) {
            float nd = edges.size() / max_deg;
            float nf = fills[idx++] / max_fill;
            to_insert.push_back({alpha * nd + (1.0f - alpha) * nf, u});
        }
        std::sort(to_insert.begin(), to_insert.end());
        for (auto& [score, u] : to_insert) {
            uint32_t bucket = (uint32_t)adj.at(u).size();
            buckets[bucket].push_front(u);
            bucket_position[u] = buckets[bucket].begin();
            heuristic_vals[u] = bucket;
        }
    } else {
        for (const auto& [u, edges] : adj) {
            uint32_t bucket = (uint32_t)edges.size();
            buckets[bucket].push_back(u);
            bucket_position[u] = std::prev(buckets[bucket].end());
            heuristic_vals[u] = bucket;
        }
    }
    min_bucket_hint = 0;
}

void Graph::clear_buckets() {
    buckets.clear();
    heuristic_vals.clear();
}

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

                add_edge_cache(u, w, uvw_weight);
                add_edge_cache(w, u, uvw_weight);

            } else if (uvw_weight < get_edge_weight(u, w)) {
                const uint64_t key_uw = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(w);
                const uint64_t key_wu = (static_cast<uint64_t>(w) << 32) | static_cast<uint64_t>(u);
                weight_map[key_uw] = uvw_weight;
                weight_map[key_wu] = uvw_weight;
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
    if (!heuristic_vals.empty()) {
        if (use_min_neighbor_degree) {
            // MND 1-hop update: only direct neighbors of v need recomputing.
            // 2-hop neighbors have slightly stale scores but the approximation
            // is close on sparse road networks and keeps runtime O(d) per step.
            for (uint32_t u : neighbors) {
                if (!adj.count(u)) continue;

                uint32_t old_score = heuristic_vals[u];
                uint32_t new_score = get_neighbor_degree_score(u);
                if (old_score == new_score) continue;

                // resize bucket array if score exceeds current size
                if (new_score >= buckets.size()) buckets.resize(new_score + 1);

                buckets[old_score].erase(bucket_position[u]);
                buckets[new_score].push_front(u);
                bucket_position[u] = buckets[new_score].begin();
                heuristic_vals[u] = new_score;
                if (new_score < min_bucket_hint) min_bucket_hint = new_score;
            }
        } else {
            for (uint32_t neighbor : neighbors) {
                const uint32_t d1 = heuristic_vals[neighbor];
                uint32_t d2;

                if (is_min_degree || (hybrid_alpha > 0.0f && hybrid_alpha < 1.0f)) {
                    d2 = adj.at(neighbor).size();
                } else {
                    d2 = get_fill(neighbor);
                }

                buckets[d1].erase(bucket_position[neighbor]);
                buckets[d2].push_front(neighbor);
                bucket_position[neighbor] = buckets[d2].begin();
                heuristic_vals[neighbor] = d2;
                if (d2 < min_bucket_hint) min_bucket_hint = d2;
            }
        }
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

uint32_t Graph::pop_next_vertex() {
    while (min_bucket_hint < buckets.size() && buckets[min_bucket_hint].empty()) {
        min_bucket_hint++;
    }

    const uint32_t v = buckets[min_bucket_hint].front();
    buckets[min_bucket_hint].pop_front();

    return v;
}

bool Graph::edge_exists(const uint32_t u, const uint32_t v) const {
    if (!weighted_graph) {
        auto it = adj.find(u);
        if (it == adj.end()) return false;
        for (const Edge& e : it->second) {
            if (e.to == v) return true;
        }
        return false;
    }
    const uint64_t edge = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
    return edge_set.contains(edge);
}

uint32_t Graph::get_edge_weight(const uint32_t u, const uint32_t v) const {
    if (u == v) return 0;
    if (!weighted_graph) {
        return edge_exists(u, v) ? 1 : UINT32_MAX;
    }
    const uint64_t key = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
    auto it = weight_map.find(key);
    if (it != weight_map.end()) return it->second;
    auto ait = adj.find(u);
    if (ait != adj.end()) {
        for (const Edge& e : ait->second) {
            if (e.to == v) return e.w;
        }
    }
    return UINT32_MAX;
}

void Graph::add_edge_cache(const uint32_t u, const uint32_t v, const uint32_t w) {
    if (weighted_graph) {
        const uint64_t edge_cache = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
        edge_set.insert(edge_cache);
        weight_map[edge_cache] = w;
    }
}

void Graph::remove_edge_cache(const uint32_t u, const uint32_t v) {
    if (weighted_graph) {
        const uint64_t edge_cache = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
        edge_set.erase(edge_cache);
        weight_map.erase(edge_cache);
    }
}

// Lexicographic BFS
std::vector<uint32_t> Graph::lex_bfs() const {
    const size_t n = max_vertex_id + 1;
    const size_t num_verts = adj.size();

    struct Partition { std::list<uint32_t> verts; };
    std::list<Partition> plist;
    plist.push_back({});

    using PIt   = std::list<Partition>::iterator;
    using VtxIt = std::list<uint32_t>::iterator;

    std::vector<PIt>    part_of(n, plist.end());
    std::vector<VtxIt>  pos_in(n);

    for (const auto& [u, _] : adj) {
        plist.front().verts.push_back(u);
        pos_in[u]  = std::prev(plist.front().verts.end());
        part_of[u] = plist.begin();
    }

    std::vector<uint32_t> order;
    order.reserve(num_verts);
    std::vector<bool> visited(n, false);

    std::unordered_map<Partition*, PIt> new_sib;
    new_sib.reserve(64);

    for (size_t i = 0; i < num_verts; i++) {
        while (!plist.empty() && plist.front().verts.empty())
            plist.pop_front();

        const uint32_t v = plist.front().verts.front();
        plist.front().verts.pop_front();
        visited[v] = true;
        order.push_back(v);

        new_sib.clear();

        for (const auto& [u, w] : adj.at(v)) {
            if (visited[u]) continue;

            PIt cur = part_of[u];
            Partition* key = &(*cur);

            auto sit = new_sib.find(key);
            if (sit == new_sib.end()) {
                PIt ins = plist.insert(cur, Partition{});
                sit = new_sib.emplace(key, ins).first;
            }

            PIt np = sit->second;
            cur->verts.erase(pos_in[u]);
            np->verts.push_back(u);
            pos_in[u]  = std::prev(np->verts.end());
            part_of[u] = np;
        }

        for (auto& [key, it] : new_sib) {
            (void)key;
            if (it->verts.empty()) plist.erase(it);
        }
    }

    return order;
}

std::vector<uint32_t> Graph::get_random_ordering() const {
    std::vector<uint32_t> ordering(max_vertex_id + 1);
    for (uint32_t u = 0; u <= max_vertex_id; u++) {
        ordering[u] = u;
    }

    std::mt19937_64 rng(std::random_device{}());
    std::ranges::shuffle(ordering, rng);

    return ordering;
}

std::tuple<Graph::TreeDecompAdj, Graph::TreeDecompBags, uint32_t> Graph::get_td(Heuristic heuristic, float alpha) {
    bool needs_buckets = (heuristic != Heuristic::LEX_BFS);
    Graph h = Graph(adj, needs_buckets);
    h.max_vertex_id = max_vertex_id;
    h.hybrid_alpha = 0.0f;
    h.weighted_graph = weighted_graph;
    h.use_min_neighbor_degree = false;

    const auto adj_size = max_vertex_id + 1;

    h.num_vertices = adj.size();
    h.adj_original_size = adj.size();

    if (heuristic == Heuristic::MIN_FILL) {
        h.populate_buckets_min_fill();
    } else if (heuristic == Heuristic::MIN_DEGREE) {
        for (const auto& [u, edges] : adj) {
            for (const auto& [to, w] : edges) {
                h.add_edge_cache(u, to, w);
            }
        }
        h.populate_buckets();
    } else if (heuristic == Heuristic::HYBRID) {
        h.populate_buckets_hybrid(alpha);
    } else if (heuristic == Heuristic::MIN_NEIGHBOR_DEGREE) {
        h.use_min_neighbor_degree = true;
        h.populate_buckets_min_neighbor_degree();
    } else {
        // LEX_BFS: populate edge_set only
        for (const auto& [u, edges] : adj) {
            for (const auto& [to, w] : edges) {
                h.add_edge_cache(u, to, w);
            }
        }
    }

    h.td_bag_edges.resize(adj_size);

    std::vector<uint32_t> ordering(adj_size, UINT32_MAX);
    parent_map.assign(adj_size, UINT32_MAX);

    td_bag_edges.resize(adj_size);
    td_weights.resize(adj_size);

    td_bags.clear();
    td_adj.clear();
    treeheight = 0;
    td_root = 1e9;

    td_adj.resize(adj_size);
    td_bags.resize(adj_size);

    if (heuristic == Heuristic::LEX_BFS) {
        std::vector<uint32_t> elim_order = h.lex_bfs();
        std::ranges::reverse(elim_order);

        for (size_t i = 0; i < elim_order.size(); i++) {
            uint32_t v = elim_order[i];
            td_bags[v] = h.get_star(v);
            h.eliminate_vertex(v, false);
            ordering[v] = i;

            if (i % static_cast<int>(adj.size() / 10) == 0)
                std::cout << "Eliminated vertex " << i << " " << 10 * i / static_cast<int>(adj.size() / 10) << "%" << std::endl;
        }
    } else {
        // MIN_DEGREE, MIN_FILL, HYBRID, MIN_NEIGHBOR_DEGREE all use pop_next_vertex()
        // Pass is_min_degree=true only for MIN_DEGREE so bucket updates use degree not fill
        const bool is_min_degree = (heuristic == Heuristic::MIN_DEGREE);

        for (size_t i = 0; i < adj.size(); i++) {
            uint32_t v = h.pop_next_vertex();
            td_bags[v] = h.get_star(v);
            h.eliminate_vertex(v, is_min_degree);
            ordering[v] = i;

            if (i % static_cast<int>(adj.size() / 10) == 0)
                std::cout << "Eliminated vertex " << i << " " << 10 * i / static_cast<int>(adj.size() / 10) << "%" << std::endl;
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
            if (td_root == static_cast<uint32_t>(1e9)) td_root = v;
            parent_map[v] = v;
            continue;
        }

        td_adj[v].push_back(min_u);
        td_adj[min_u].push_back(v);
        parent_map[v] = min_u;
    }

    for (uint32_t v : adj | std::views::keys) {
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

    // compute treeheight via BFS from root on td_adj
    if (td_root != static_cast<uint32_t>(1e9)) {
        std::queue<std::pair<uint32_t, uint32_t>> bfs_q;
        std::unordered_set<uint32_t> bfs_visited;
        bfs_q.push({td_root, 0});
        bfs_visited.insert(td_root);
        treeheight = 0;
        while (!bfs_q.empty()) {
            auto [node, depth] = bfs_q.front();
            bfs_q.pop();
            treeheight = std::max(treeheight, (size_t)depth);
            for (uint32_t neighbor : td_adj.at(node)) {
                if (!bfs_visited.contains(neighbor)) {
                    bfs_visited.insert(neighbor);
                    bfs_q.push({neighbor, depth + 1});
                }
            }
        }
    }

    // free temporary structures to reduce peak memory
    { TreeDecompBagEdges tmp; h.td_bag_edges.swap(tmp); }
    td_bag_edges.clear(); td_bag_edges.shrink_to_fit();
    td_weights.clear(); td_weights.shrink_to_fit();

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
        uint32_t parent = parent_map[current];
        if (parent == UINT32_MAX || parent == current) break;
        path.push_back(current);
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
    return treeheight;
}