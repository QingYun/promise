#include "../promise.h"
#include <gtest/gtest.h>

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

TEST(PromiseTest, FulfillWithScalarValue) {
  {
    Promise<int> p{};
    auto fulfiller = p.getFulfiller();
    fulfiller.fulfill(123);
    EXPECT_EQ(123, p.wait());
  }

  {
    Fulfiller<int> fulfiller;
    Promise<int> p{ [&fulfiller](Fulfiller<int> f) {
      fulfiller = std::move(f);
    }};
    fulfiller.fulfill(123);
    EXPECT_EQ(123, p.wait());
  }
}

TEST(PromiseTest, FulfillWithVoid) {
  {
    Promise<void> p{};
    auto fulfiller = p.getFulfiller();
    fulfiller.fulfill();
    EXPECT_NO_THROW(p.wait());
  }

  {
    Fulfiller<void> fulfiller;
    Promise<void> p{ [&fulfiller](Fulfiller<void> f) {
      fulfiller = std::move(f);
    } };
    fulfiller.fulfill();
    EXPECT_NO_THROW(p.wait());
  }
}

TEST(PromiseTest, Reject) {
  Promise<int> p{};
  auto fulfiller = p.getFulfiller();
  fulfiller.reject<std::runtime_error>("reject_test");
  try {
    p.wait();
  }
  catch (const std::runtime_error& e) {
    EXPECT_STREQ("reject_test", e.what());
  }
}

TEST(PromiseTest, AutoReject) {
  {
    Promise<void> p{};
    {
      p.getFulfiller();
    }
    try {
      p.wait();
    }
    catch (const std::logic_error& e) {
      EXPECT_STREQ("~Fulfiller", e.what());
    }
  }

  {
    Promise<int> p{[](Fulfiller<int>){}};
    try {
      p.wait();
    }
    catch (const std::logic_error& e) {
      EXPECT_STREQ("~Fulfiller", e.what());
    }
  }
}

TEST(PromiseTest, MovePromise) {
  auto CheckResult = [] (Promise<int> promise) {
    EXPECT_EQ(123, promise.wait());
  };

  auto FulfillPromise = [] (Fulfiller<int> fulfiller) {
    fulfiller.fulfill(123);
  };

  Promise<int> p{};
  auto fulfiller = p.getFulfiller();
  FulfillPromise(std::move(fulfiller));
  CheckResult(std::move(p));
}

TEST(PromiseTest, MakePromise) {
  {
    Promise<void> p = makePromise();
    EXPECT_NO_THROW(p.wait());
  }

  {
    Promise<int> p = makePromise(123);
    EXPECT_EQ(123, p.wait());
  }

  {
    Promise<void> p = makePromise(
        std::make_exception_ptr(std::runtime_error("make_promise_test")));
    try {
      p.wait();
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("make_promise_test", e.what());
    }
  }

  {
    Promise<int, NoEventLoop, MultiThreaded> p =
        makePromise<int, NoEventLoop, MultiThreaded>(
            std::make_exception_ptr(std::runtime_error("make_promise_test")));
    try {
      p.wait();
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("make_promise_test", e.what());
    }
  }
}

TEST(PromiseTest, ThenFulfill) {
  {
    MoveOnly mo{123};
    Promise<MoveOnly> p(partiallyApply([](MoveOnly mo, Fulfiller<MoveOnly> f) {
                                         f.fulfill(std::move(mo));
                                       },
                                       std::move(mo)));

    p.then([](MoveOnly mo) {
             EXPECT_EQ(123, mo.value_);
             mo.value_ = 456;
             return std::move(mo);
           }).then([](MoveOnly mo) { EXPECT_EQ(456, mo.value_); });
  }

  {
    auto p = makePromise(MoveOnly{123});
    p.then(nullptr, [](std::exception_ptr e) { return MoveOnly{456}; })
        .then([](MoveOnly mo) { EXPECT_EQ(123, mo.value_); });
  }
}

TEST(PromiseTest, ThenReject) {
  {
    auto p = makePromise(
        std::make_exception_ptr(std::runtime_error("promise_test")));
    p.then([]() { return MoveOnly{456}; }, [](std::exception_ptr e) {
             try {
               std::rethrow_exception(e);
             }
             catch (const std::runtime_error& err) {
               EXPECT_STREQ("promise_test", err.what());
             }

             return MoveOnly{123};
           }).then([](MoveOnly mo) { EXPECT_EQ(123, mo.value_); });
  }
}

TEST(PromiseTest, ThenProgress) {
  {
    MoveOnly count{0};
    Promise<MoveOnly> p{};
    auto fulfiller = p.getFulfiller();

    auto n = p.then([](MoveOnly mo, std::tuple<MoveOnly>& attachment) {
             auto& count = std::get<0>(attachment);
             count.value_ += mo.value_;
             return std::move(count);
           },
           nullptr, [](MoveOnly mo, std::tuple<MoveOnly>& attachment) {
                      std::get<0>(attachment).value_ += mo.value_;
                    },
           std::move(count));

    for (int i = 0; i < 5; ++i) fulfiller.progress(i);
    fulfiller.progress(MoveOnly{10});
    fulfiller.fulfill(MoveOnly{22});

    EXPECT_EQ(42, n.wait().value_);
  }
}