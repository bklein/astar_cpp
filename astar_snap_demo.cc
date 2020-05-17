#include <cstdlib>
#include <chrono>
#include <iostream>

#include "astar.hpp"
#include "snap_graph.hpp"
#include "random_uniform_generator.hpp"

using namespace bk;
using Node = UndirectedGraph::Node;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " dataset.txt" << std::endl;
    return EXIT_FAILURE;
  }

  const std::filesystem::path dataset_path(argv[1]);
  const auto maybe_graph = ParseFromFile(dataset_path);
  if (!maybe_graph.has_value()) {
    std::cerr << "Failed to parse dataset: " << dataset_path << std::endl;
    return EXIT_FAILURE;
  }

  const auto& graph = *maybe_graph;

  std::cout << "Loaded graph with "
    << graph.num_nodes() << " nodes and "
    << graph.num_edges() << " edges"
    << std::endl;

  const auto nodes = graph.nodes();
  RandomUniformGenerator<Node> rand(0, nodes.size() - 1);

  const auto start = nodes.at(rand.next());
  const auto goal = nodes.at(rand.next());

  auto neighors =
    [&graph] (const Node& a) -> std::vector<Node> {
      const auto nodes = graph.connections(a);
      if (nodes) {
        return *nodes;
      } else {
        return {};
      }
    };

  auto h =
    [] (const Node&) -> double {
      // Since the dataset does not contain true spatial information
      // return a fake constant heuristic, making this equivilant
      // to Dijkstra
      return 1.0;
    };

  auto edge_cost =
    [] (const Node&, const Node&) -> double {
      // Since the dataset does not contain edge costs
      // we will use a simple constant cost
      return 1.0;
    };

  std::cout << "start: " << start << std::endl;
  std::cout << " goal: " << goal << std::endl;

  const auto maybe_path =
    [&] {
      const auto t0 = std::chrono::high_resolution_clock::now();
      const auto maybe_path = AStar(start, goal, neighors, edge_cost, h);
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

  return EXIT_SUCCESS;
}
