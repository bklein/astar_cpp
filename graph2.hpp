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

// XXX Adapt to use this instead
// it has spatial data!
// https://www.cs.utah.edu/~lifeifei/SpatialDataset.htm

template <class T> using observer_ptr = T*;

namespace normalized_coordinate {

using NodeId = int64_t;

struct NodeEdge {
  NodeEdge(NodeId id, double cannonical_l2_cost)
    : id(id), cannonical_l2_cost(cannonical_l2_cost) {}
  NodeId id;
  double cannonical_l2_cost;
};

struct Node {
  Node(double x, double y)
    : x(x), y(y), edges() {}
  double x;
  double y;
  std::vector<NodeEdge> edges;
};

class Graph {
 public:
  Graph(std::unordered_map<NodeId, Node>&& nodes)
    : nodes_(std::move(nodes))
    , n_edges_(0) {

    for (const auto& [id, node] : nodes_) {
      n_edges_ += node.edges.size();
    }

    // assumes balanced
    assert(n_edges_ % 2 == 0);
    n_edges_ /= 2;
  }

  std::size_t num_nodes() const {
    return nodes_.size();
  }

  std::size_t num_edges() const {
    return n_edges_;
  }

  std::vector<NodeId> node_ids() const {
    std::vector<NodeId> node_ids;
    for (const auto& [id, node] : nodes_) {
      node_ids.emplace_back(id);
    }
    return node_ids;
  }

  std::vector<NodeId> neighbors(NodeId id) const {
    auto it = nodes_.find(id);
    std::vector<NodeId> node_ids;
    if (it != nodes_.end()) {
      const auto& node = it->second;
      for (const auto n : node.edges) {
        node_ids.emplace_back(n.id);
      }
    }
    return node_ids;
  }

  std::optional<Node> node(NodeId id) const {
    auto it = nodes_.find(id);
    if (it != nodes_.end()) {
      return std::make_optional(it->second);
    } else {
      return std::nullopt;
    }
  }


 private:
  std::unordered_map<NodeId, Node> nodes_;
  std::size_t n_edges_;
};

std::optional<std::unordered_map<NodeId, Node>> ParseNodesFromFile(
    const std::filesystem::path& cnode_filename) {
  std::ifstream ifs(cnode_filename.string());
  if (!ifs.is_open())
    return std::nullopt;

  std::unordered_map<NodeId, Node> nodes;

  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    NodeId id;
    iss >> id;
    if (iss.fail())
      return std::nullopt;
    double x;
    iss >> x;
    if (iss.fail())
      return std::nullopt;
    double y;
    iss >> y;
    if (iss.fail())
      return std::nullopt;
    const auto [it, inserted] = nodes.emplace(id, Node(x, y));
    (void)it;
    assert(inserted);
  }

  return nodes;
}

using RawEdge = std::tuple<NodeId, NodeId, double>;

std::optional<std::vector<RawEdge>> ParseEdgesFromFile(
    const std::filesystem::path& cedge_filename) {
  std::ifstream ifs(cedge_filename.string());
  if (!ifs.is_open())
    return std::nullopt;

  std::vector<RawEdge> raw_edges;

  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    NodeId edge_id;
    iss >> edge_id;
    if (iss.fail())
      return std::nullopt;
    NodeId from_id;
    iss >> from_id;
    if (iss.fail())
      return std::nullopt;
    NodeId to_id;
    iss >> to_id;
    if (iss.fail())
      return std::nullopt;
    double l2_cost;
    iss >> l2_cost;
    if (iss.fail())
      return std::nullopt;
    raw_edges.emplace_back(from_id, to_id, l2_cost);

  }

  return std::make_optional(std::move(raw_edges));
}

std::optional<Graph> ParseFromFiles(
    const std::filesystem::path& cnode_filename,
    const std::filesystem::path& cedge_filename) {

  auto nodes = ParseNodesFromFile(cnode_filename);
  if (!nodes)
    return std::nullopt;

  auto edges = ParseEdgesFromFile(cedge_filename);
  if (!edges)
    return std::nullopt;

  for (const auto& [from_id, to_id, l2_cost] : *edges) {
    nodes->at(from_id).edges.emplace_back(to_id, l2_cost);
    nodes->at(to_id).edges.emplace_back(from_id, l2_cost);
  }

  return std::make_optional<Graph>(std::move(*nodes));
}

bool VerifyPath(const Graph& graph, const std::vector<NodeId>& path) {
  for (std::size_t i=1; i<path.size(); ++i) {
    const auto from_id = path[i-1];
    const auto to_id = path[i];
    const auto maybe_from_node = graph.node(from_id);
    if (!maybe_from_node.has_value())
      return false;

    const auto& edges = maybe_from_node.value().edges;
    bool valid_edge =
      std::find_if(
          edges.begin(), edges.end(),
          [to_id] (const auto& edge) {
            return edge.id == to_id;
          })
      != edges.end();
    if (!valid_edge)
      return false;
  }
  return true;
}

}  // namespace normalized_coordinate
}  // namespace bk
