#include "exception_or.h"
#include <gtest/gtest.h>

using namespace promise::_;

TEST(ExceptionOrTest, Empty) {
  ExceptionOr<int> eoi{};
  EXPECT_FALSE(eoi.IsException());
  EXPECT_FALSE(eoi.IsValue());
  EXPECT_TRUE(eoi.IsEmpty());
}

TEST(ExceptionOrTest, SaveException) {
  {
    ExceptionOr<int> eoi{
        std::make_exception_ptr(std::runtime_error("exception"))};
    EXPECT_TRUE(eoi.IsException());
    EXPECT_FALSE(eoi.IsValue());
    EXPECT_FALSE(eoi.IsEmpty());
    try {
      std::rethrow_exception(eoi.GetException());
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("exception", e.what());
      EXPECT_TRUE(eoi.IsEmpty());
    }
  }

  {
    ExceptionOr<int> eoi{};
    EXPECT_TRUE(eoi.IsEmpty());
    eoi.SetException(std::make_exception_ptr(std::runtime_error("exception")));
    EXPECT_TRUE(eoi.IsException());
    EXPECT_FALSE(eoi.IsValue());
    EXPECT_FALSE(eoi.IsEmpty());
    try {
      std::rethrow_exception(eoi.GetException());
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("exception", e.what());
      EXPECT_TRUE(eoi.IsEmpty());
    }
  }
}

TEST(ExceptionOrTest, SaveScalarValue) {
  {
    ExceptionOr<int> eoi{123};
    EXPECT_TRUE(eoi.IsValue());
    EXPECT_FALSE(eoi.IsException());
    EXPECT_FALSE(eoi.IsEmpty());
    EXPECT_EQ(123, eoi.GetValue());
    EXPECT_TRUE(eoi.IsEmpty());
  }

  {
    ExceptionOr<int> eoi{};
    EXPECT_TRUE(eoi.IsEmpty());
    eoi.SetValue(123);
    EXPECT_TRUE(eoi.IsValue());
    EXPECT_FALSE(eoi.IsException());
    EXPECT_FALSE(eoi.IsEmpty());
    EXPECT_EQ(123, eoi.GetValue());
    EXPECT_TRUE(eoi.IsEmpty());
  }
}

TEST(ExceptionOrTest, SaveClassValue) {
  struct C {
    int value_;
    explicit C(int value) : value_(value) {}

    C(const C&) = delete;
    C& operator=(const C&) = delete;

    C(C&& other) : value_(other.value_) {}
    C& operator=(C&& other) {
      value_ = other.value_;
      return *this;
    }
  };

  {
    ExceptionOr<C> eoc{ C{123} };
    EXPECT_TRUE(eoc.IsValue());
    EXPECT_FALSE(eoc.IsException());
    C c = eoc.GetValue();
    EXPECT_EQ(123, c.value_);
    EXPECT_TRUE(eoc.IsEmpty());
  }

  {
    ExceptionOr<C> eoc{};
    EXPECT_TRUE(eoc.IsEmpty());
    eoc.SetValue(C{ 123 });
    EXPECT_TRUE(eoc.IsValue());
    EXPECT_FALSE(eoc.IsException());
    EXPECT_FALSE(eoc.IsEmpty());
    C c = eoc.GetValue();
    EXPECT_EQ(123, c.value_);
    EXPECT_TRUE(eoc.IsEmpty());
  }
}