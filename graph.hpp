#pragma once
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace bk {

using Node = int64_t;

using UndirectedGraph = std::unordered_map<Node, std::vector<Node>>;

std::optional<UndirectedGraph> LoadFromFile(const std::filesystem::path& filename) {
  std::ifstream ifs(filename.string());
  if (!ifs.is_open())
    return std::nullopt;

  UndirectedGraph graph;
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

    ++line_no;
  }

  return std::make_optional(std::move(graph));
}

bool VerifyPath(const UndirectedGraph& graph, const std::vector<Node>& path) {
  for (std::size_t i=1; i<path.size(); ++i) {
    const auto from = path[i-1];
    const auto to = path[i];
    const auto& edges = graph.at(from);
    bool valid_edge = std::find(edges.begin(), edges.end(), to) != edges.end();
    if (!valid_edge)
      return false;
  }
  return true;
}

}  // namespace bk
