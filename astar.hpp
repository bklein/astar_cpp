#pragma once
#include <optional>
#include <queue>
#include <unordered_map>

namespace bk {

template <class Node
         ,class NeighborFunc // (const Node&) -> std::vector<Node>
         ,class EdgeCost // (const Node&, const Node&) -> double
         ,class HFunc> // (const Node&) -> double
std::optional<std::vector<Node>>
AStar(
    const Node& start,
    const Node& goal,
    NeighborFunc find_neighbors,
    EdgeCost edge_cost,
    HFunc h) {
  using NodeScore = std::pair<Node, double>;
  auto comp =
    [] (const NodeScore& a, const NodeScore& b) -> bool {
      return a.second > b.second;
    };
  std::priority_queue<NodeScore, std::vector<NodeScore>, decltype(comp)> open_set(comp);

  std::unordered_map<Node, double> gscores{{start, 0.0}};
  std::unordered_map<Node, double> fscores{{start, h(start)}};
  std::unordered_map<Node, Node> came_from;

  open_set.push(std::make_pair(start, 0.0));

  while (!open_set.empty()) {
    const auto [current, score] = open_set.top();
    open_set.pop();
    if (current == goal) {
      std::vector<Node> path{current};
      auto next = came_from.find(current);
      while (next != came_from.end()) {
        path.emplace_back(next->second);
        next = came_from.find(next->second);
      }
      return std::make_optional(std::move(path));
    }

    const auto neighors = find_neighbors(current);
    const auto gscore_current = gscores.at(current);
    for (const auto& neighbor : neighors) {
      const auto delta_cost = edge_cost(current, neighbor);
      const auto new_gscore = gscore_current + delta_cost;
      auto it = gscores.find(neighbor);
      const bool replace_node = it == gscores.end() || new_gscore < it->second;
      if (replace_node) {
        came_from[neighbor] = current;
        gscores[neighbor] = new_gscore;
        const auto hscore = h(neighbor);
        const auto fscore = new_gscore + hscore;
        fscores[neighbor] = fscore;
        open_set.push(std::make_pair(neighbor, fscore));
      }
    }
  }

  return std::nullopt;
}

}  // namespace bk
