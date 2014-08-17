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

template <typename T>
Promise<T> rejected() {
  return makePromise<T>(std::make_exception_ptr(std::runtime_error{ "" }));
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

// When rejected, a promise: must not transition to any other state.
TEST(PromiseAPlusTest, Test_2_1_3_1) {
  // trying to reject then immediately fulfill
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_rejected_called = false;

    auto new_promise = promise.then(
      [&on_rejected_called]() {
        EXPECT_EQ(false, on_rejected_called);
      },
      [&on_rejected_called](std::exception_ptr) {
        on_rejected_called = true;
      });

    reject(fulfiller, std::make_exception_ptr(std::runtime_error{""}));
    resolve(fulfiller);
  }

  // trying to reject then fulfill, delayed
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_rejected_called = false;

    auto new_promise = promise.then(
      [&on_rejected_called]() {
        EXPECT_EQ(false, on_rejected_called);
      },
      [&on_rejected_called](std::exception_ptr) {
        on_rejected_called = true;
      });

    std::thread{[&fulfiller]() {
                  reject(fulfiller,
                         std::make_exception_ptr(std::runtime_error{""}));
                  resolve(fulfiller);
                }}.join();
  }

  // trying to reject immediately then fulfill delayed
  {
    auto promise = Promise<void>{};
    auto fulfiller = promise.getFulfiller();
    bool on_rejected_called = false;

    auto new_promise = promise.then(
      [&on_rejected_called]() {
        EXPECT_EQ(false, on_rejected_called);
      },
      [&on_rejected_called](std::exception_ptr) {
        on_rejected_called = true;
      });

    reject(fulfiller, std::make_exception_ptr(std::runtime_error{ "" }));
    std::thread{[&fulfiller]() { resolve(fulfiller); }}.join();
  }
}

// Both "onFulfilled" and "onRejected" are optional arguments

// If "onFulfilled" is a nullptr, it must be ignored
TEST(PromiseAPlusTest, Test_2_2_1_1) {
  ::testing::StaticAssertTypeEq<default_on_fulfilled_t, std::nullptr_t>();

  // applied to a directly-rejected promise
  {
    bool on_rejected_called = false;
    auto promise = rejected<void>().then(
        nullptr, [&on_rejected_called](std::exception_ptr) {
          on_rejected_called = true;
        });

    EXPECT_EQ(true, on_rejected_called);
  }

  // applied to a promise rejected and then chained off of
  {
    bool on_rejected_called = false;
    auto promise = rejected<void>().then([]() {}).then(
        nullptr, [&on_rejected_called](std::exception_ptr) {
          on_rejected_called = true;
        });

    EXPECT_EQ(true, on_rejected_called);
  }
}

// If "onRejected" is a nullptr, it must be ignored
TEST(PromiseAPlusTest, Test_2_2_1_2) {
  ::testing::StaticAssertTypeEq<default_on_rejected_t, std::nullptr_t>();

  // applied to a directly-fulfilled promise
  {
    bool on_fulfilled_called = false;
    auto promise = makePromise<MoveOnly>(MoveOnly{42})
                       .then([&on_fulfilled_called](MoveOnly mo) {
                               EXPECT_EQ(42, mo.value_);
                               on_fulfilled_called = true;
                             },
                             nullptr);

    EXPECT_EQ(true, on_fulfilled_called);
  }

  // applied to a promise fulfilled and then chained off of
  {
    bool on_fulfilled_called = false;
    auto promise =
        makePromise<MoveOnly>(MoveOnly{42})
            .then(nullptr, [](std::exception_ptr) { return MoveOnly{0}; })
            .then([&on_fulfilled_called](MoveOnly mo) {
                    EXPECT_EQ(42, mo.value_);
                    on_fulfilled_called = true;
                  },
                  nullptr);

    EXPECT_EQ(true, on_fulfilled_called);
  }
}