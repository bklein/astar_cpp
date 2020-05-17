#pragma once
#include <cassert>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace bk {

template <class T> using observer_ptr = T*;

class UndirectedGraph {
 public:
  using Node = int64_t;
  explicit UndirectedGraph(std::unordered_map<Node, std::vector<Node>>&& nodes_to_edges)
    : nodes_to_edges_(std::move(nodes_to_edges))
    , n_edges_(0) {

    for (const auto& [from, tos] : nodes_to_edges_) {
      n_edges_ += tos.size();
    }

    // assumes balanced
    assert(n_edges_ % 2 == 0);
    n_edges_ /= 2;
  }

  std::size_t num_nodes() const {
    return nodes_to_edges_.size();
  }

  std::size_t num_edges() const {
    return n_edges_;
  }

  std::vector<Node> nodes() const {
    std::vector<Node> nodes;
    for (const auto& [from, tos] : nodes_to_edges_) {
      nodes.emplace_back(from);
    }
    return nodes;
  }

  observer_ptr<const std::vector<Node>> connections(Node n) const {
    auto it = nodes_to_edges_.find(n);
    if (it == nodes_to_edges_.end()) {
      return nullptr;
    } else {
      return &it->second;
    }
  }

 private:
  std::unordered_map<Node, std::vector<Node>> nodes_to_edges_;
  std::size_t n_edges_;
};

std::optional<UndirectedGraph> ParseFromFile(const std::filesystem::path& filename) {
  std::ifstream ifs(filename.string());
  if (!ifs.is_open())
    return std::nullopt;

  using Node = UndirectedGraph::Node;
  std::unordered_map<Node, std::vector<Node>> graph;

  std::string line;
  int line_no = 1;
  while (std::getline(ifs, line)) {
    if (line.empty())
      continue;
    if (line[0] == '#')
      continue;

    std::istringstream iss(line);
    int64_t from_node;
    iss >> from_node;
    if (iss.fail()) {
      std::cerr << "Failed to parse line " << line_no << ": " << line << std::endl;
      return std::nullopt;
    }

    int64_t to_node;
    iss >> to_node;
    if (iss.fail()) {
      std::cerr << "Failed to parse line " << line_no << ": " << line << std::endl;
      return std::nullopt;
    }

    graph[from_node].emplace_back(to_node);
    graph[to_node].emplace_back(from_node);

    ++line_no;
  }

  return std::make_optional<UndirectedGraph>(std::move(graph));
}

bool VerifyPath(const UndirectedGraph& graph, const std::vector<UndirectedGraph::Node>& path) {
  for (std::size_t i=1; i<path.size(); ++i) {
    const auto from = path[i-1];
    const auto to = path[i];
    const auto edges = graph.connections(from);
    if (!edges)
      return false;
    bool valid_edge = std::find(edges->begin(), edges->end(), to) != edges->end();
    if (!valid_edge)
      return false;
  }
  return true;
}

}  // namespace bk
