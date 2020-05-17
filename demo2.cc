#include <cstdlib>
#include <chrono>
#include <iostream>

#include "astar.hpp"
#include "graph2.hpp"
#include "random_uniform_generator.hpp"

using namespace bk;
using namespace bk::normalized_coordinate;

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " dataset_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string dataset_name(argv[1]);
  const fs::path cnode_filename(dataset_name + std::string(".cnode"));
  const fs::path cedge_filename(dataset_name + std::string(".cedge"));
  const auto maybe_graph = ParseFromFiles(cnode_filename, cedge_filename);
  if (!maybe_graph.has_value()) {
    std::cerr << "Failed to parse dataset: " << dataset_name << std::endl;
    return EXIT_FAILURE;
  }

  const auto& graph = *maybe_graph;

  std::cout << "Loaded graph with "
    << graph.num_nodes() << " nodes and "
    << graph.num_edges() << " edges"
    << std::endl;

#if 1
  const auto node_ids = graph.node_ids();
  RandomUniformGenerator<std::size_t> rand(0, node_ids.size() - 1);

  const auto start_id = node_ids.at(rand.next());
  const auto goal_id = node_ids.at(rand.next());

  auto neighbors =
    [&graph] (const NodeId& id) -> std::vector<NodeId> {
      return graph.neighbors(id);
    };

  auto h =
    [&graph, goal_id] (NodeId id) -> double {
      const auto current_node = graph.node(id).value();
      const auto goal_node = graph.node(goal_id).value();
      const double delta_x = current_node.x - goal_node.x;
      const double delta_y = current_node.y - goal_node.y;
      const double distance = delta_x * delta_x + delta_y * delta_y;
      return distance;
    };

  auto edge_cost =
    [&graph] (NodeId a, NodeId b) -> double {
      const auto edges = graph.node(a).value().edges;
      auto it = std::find_if(
          edges.begin(), edges.end(),
          [b] (const auto& edge) {
            return edge.id == b;
          });
      if (it == edges.end()) {
        std::cerr << "Data is inconsistent" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      return it->cannonical_l2_cost;
    };

  std::cout << "start: " << start_id << std::endl;
  std::cout << " goal: " << goal_id << std::endl;

  const auto maybe_path =
    [&] {
      const auto t0 = std::chrono::high_resolution_clock::now();
      const auto maybe_path = AStar(start_id, goal_id, neighbors, edge_cost, h);
      const auto t1 = std::chrono::high_resolution_clock::now();
      const auto duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
      std::cout << "A* took " << duration.count() << " secs." << std::endl;
      return maybe_path;
    } ();

  if (maybe_path.has_value()) {
    const auto& path = *maybe_path;
    if (!VerifyPath(graph, path)) {
      std::cerr << "Path failed verification" << std::endl;
    }
    std::cout << "path, length = " << path.size() << std::endl;
    constexpr bool verbose = false;
    if (verbose) {
      for (const auto& node : path) {
        std::cout << node << std::endl;
      }
    }
  } else {
    std::cerr << "AStar failed to find path" << std::endl;
  }
#endif

  return EXIT_SUCCESS;
}
