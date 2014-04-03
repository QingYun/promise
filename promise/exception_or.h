#pragma once
#include <type_traits>
#include <exception>
#include <new>
#include <cassert>

namespace promise {

namespace _ {

template <
    typename T,
    bool TIsLarger = (sizeof(typename std::aligned_storage<
                          sizeof(T), std::alignment_of<T>::value>::type) >
                      sizeof(typename std::aligned_storage<
                          sizeof(std::exception_ptr),
                          std::alignment_of<std::exception_ptr>::value>::type))>
struct Storage_;

template <typename T>
struct Storage_<T, true> {
  using Type = typename std::aligned_storage<sizeof(T),
                                             std::alignment_of<T>::value>::type;
};

template <typename T>
struct Storage_<T, false> {
  using Type = typename std::aligned_storage<
      sizeof(std::exception_ptr),
      std::alignment_of<std::exception_ptr>::value>::type;
};

template <typename T>
class Storage {
  typename Storage_<T>::Type storage_;

 public:
  template <typename T>
  void Set(T&& v) {
    new (&storage_) T{std::move(v)};
  }
  template <typename T>
  T& As() {
    return *reinterpret_cast<T*>(&storage_);
  }
};

template <typename T>
class ExceptionOr {
  Storage<T> storage_;

  enum class State { Empty, Exception, Value } state_;

 public:
  ExceptionOr() : state_(State::Empty) {}
  ExceptionOr(T&& value) : state_(State::Value) {
    storage_.Set(std::move(value));
  }
  ExceptionOr(std::exception_ptr exception) : state_(State::Exception) {
    storage_.Set(std::move(exception));
  }

  bool IsEmpty() const { return state_ == State::Empty; }
  bool IsException() const { return state_ == State::Exception; }
  bool IsValue() const { return state_ == State::Value; }

  void SetException(std::exception_ptr exception) {
    assert(IsEmpty());
    storage_.Set(std::move(exception));
    state_ = State::Exception;
  }
  void SetValue(T&& value) {
    assert(IsEmpty());
    storage_.Set(std::move(value));
    state_ = State::Value;
  }

  std::exception_ptr GetException() {
    state_ = State::Empty;
    return std::move(storage_.As<std::exception_ptr>());
  }
  T GetValue() {
    state_ = State::Empty;
    return std::move(storage_.As<T>());
  }
};

}  // namespace _

}  // namespace promise