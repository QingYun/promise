#include "promise.h"
#include <gtest/gtest.h>

using namespace promise;

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