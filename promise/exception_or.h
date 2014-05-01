#pragma once
#include "common.h"
#include <type_traits>
#include <exception>
#include <new>
#include <memory>
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
  T* As() {
    return reinterpret_cast<T*>(&storage_);
  }
};

template <typename T>
class ExceptionOr {
  Storage<T> storage_;

  enum class State { Empty, Exception, Value } state_;

 public:
  ExceptionOr() : state_{State::Empty} {}
  ExceptionOr(T&& value) : ExceptionOr{} { setValue(std::move(value)); }
  ExceptionOr(std::exception_ptr exception) : ExceptionOr{} {
    setException(std::move(exception));
  }

  ~ExceptionOr() {
    using namespace std;
    if (isException())
      storage_.As<exception_ptr>()->~exception_ptr();
    else if (isValue())
      storage_.As<T>()->~T();
  }

  PROMISE_DISALLOW_COPY(ExceptionOr);
  PROMISE_DISALLOW_MOVE(ExceptionOr);

  bool isEmpty() const { return state_ == State::Empty; }
  bool isException() const { return state_ == State::Exception; }
  bool isValue() const { return state_ == State::Value; }

  void setException(std::exception_ptr exception) {
    assert(isEmpty());
    new (storage_.As<void>()) std::exception_ptr{std::move(exception)};
    state_ = State::Exception;
  }
  template<typename... U>
  void setValue(U&&... value) {
    assert(isEmpty());
    new (storage_.As<void>()) T{std::forward<U>(value)...};
    state_ = State::Value;
  }

  std::exception_ptr getException() {
    state_ = State::Empty;
    return std::move(*storage_.As<std::exception_ptr>());
  }
  T getValue() {
    state_ = State::Empty;
    return std::move(*storage_.As<T>());
  }
};

}  // namespace _

}  // namespace promise