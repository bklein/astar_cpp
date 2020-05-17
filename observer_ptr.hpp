#pragma once

namespace bk {

template <class W>
class observer_ptr {
 public:
  using element_type = W;

  constexpr observer_ptr() noexcept : observer_ptr(nullptr) {}
  explicit observer_ptr(element_type* p) noexcept : w(p) {}

  observer_ptr(const observer_ptr& other) = default;
  observer_ptr(observer_ptr&& other) = default;

  observer_ptr& operator=(const observer_ptr& other) = default;
  observer_ptr& operator=(observer_ptr&& other) = default;

  constexpr element_type* release() noexcept {
    auto p = w;
    w = nullptr;
    return p;
  }

  constexpr void reset(element_type* p = nullptr) noexcept {
    w = p;
  }

  constexpr void swap(observer_ptr& other) noexcept {
    std::swap(w, other.w);
  }

  constexpr element_type* get() const noexcept {
    return w;
  }

  constexpr explicit operator bool() const noexcept {
    return w != nullptr;
  }

  constexpr std::add_lvalue_reference_t<element_type> operator*() const {
    return *w;
  }

  constexpr element_type* operator->() const noexcept {
    return w;
  }

 private:
  element_type* w;
};

}  // namespace bk
