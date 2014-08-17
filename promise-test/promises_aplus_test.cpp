#include "../promise.h"
#include <gtest/gtest.h>
#include <thread>

using namespace promise;

struct MoveOnly {
  int value_;
  explicit MoveOnly(int value) : value_(value) {}

  PROMISE_DISALLOW_COPY(MoveOnly);

  MoveOnly(MoveOnly&& other) : value_(other.value_) {}
  MoveOnly& operator=(MoveOnly&& other) {
    value_ = other.value_;
    return *this;
  }
};

template <typename F, typename... V>
void resolve(F&& f, V&&... v) {
  try {
    f.fulfill(std::forward<V>(v)...);
  }
  catch (...) {
  }
}

template <typename F, typename V>
void reject(F&& f, V&& v) {
  try {
    f.reject(std::forward<V>(v));
  }
  catch (...) {
  }
}

// When fulfilled, a promise: must not transition to any other state.
TEST(PromiseAPlusTest, Test_2_1_2_1) {
  // trying to fulfill then immediately reject
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_fulfilled_called = false;

    auto new_promise =
        promise.then([&on_fulfilled_called]() { on_fulfilled_called = true; },
                     [&on_fulfilled_called](std::exception_ptr) {
          EXPECT_EQ(false, on_fulfilled_called);
        });

    resolve(fulfiller);
    reject(fulfiller, std::make_exception_ptr(std::runtime_error{""}));
  }

  // trying to fulfill then reject, delayed
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_fulfilled_called = false;

    auto new_promise =
        promise.then([&on_fulfilled_called]() { on_fulfilled_called = true; },
                     [&on_fulfilled_called](std::exception_ptr) {
          EXPECT_EQ(false, on_fulfilled_called);
        });

    std::thread{[&fulfiller]() {
                  resolve(fulfiller);
                  reject(fulfiller,
                         std::make_exception_ptr(std::runtime_error{""}));
                }}.join();
  }

  // trying to fulfill immediately then reject delayed
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_fulfilled_called = false;

    auto new_promise =
        promise.then([&on_fulfilled_called]() { on_fulfilled_called = true; },
                     [&on_fulfilled_called](std::exception_ptr) {
          EXPECT_EQ(false, on_fulfilled_called);
        });

    resolve(fulfiller);
    std::thread{[&fulfiller]() {
                  reject(fulfiller,
                         std::make_exception_ptr(std::runtime_error{""}));
                }}.join();
  }
}
