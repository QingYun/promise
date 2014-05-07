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