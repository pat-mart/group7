#ifndef MCSPermutationStrategy_h
#define MCSPermutationStrategy_h

#include <unordered_set>
#include "PermutationStrategy.h"

// Implements Maximum Cardinality Search permutation strategy
// At each step: pick vertex with largest number of already
// eliminated neighbors.

class MCSPermutationStrategy : public PermutationStrategy {

public:

  // Recompute weights after selecting a node
  void recompute(std::unordered_set<unsigned long> eliminated,
                 Graph& graph,
                 unsigned long selected_node)
  {
    // Increase weight of each uneliminated neighbor
    for (auto neighbor : graph.get_neighbors(selected_node)) {

      if (eliminated.count(neighbor) == 0) {

        node_type nstruct;
        nstruct.id  = neighbor;
        nstruct.val = (*queue_nodes[neighbor]).val + 1;

        queue_nodes[neighbor] = queue.push(nstruct);
      }
    }
  }

protected:

  // Initial statistic = 0 for all nodes
  unsigned long compute_statistic(unsigned long,
                                  Graph&,
                                  bool init = false)
  {
    return 0;
  }
};

#endif /* MCSPermutationStrategy_h */