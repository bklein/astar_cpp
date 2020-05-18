#pragma once
#include <limits>
#include <random>

namespace bk {

template <class T = int>
class RandomUniformGenerator {
 public:
  static_assert(std::numeric_limits<T>::is_integer);

  RandomUniformGenerator(T lo, T hi)
    : gen_(std::random_device{}())
    , dist_(lo, hi)
  {}

  T next() {
    return dist_(gen_);
  }

 private:
  std::mt19937 gen_;
  std::uniform_int_distribution<T> dist_;
};


}  // namespace bk
