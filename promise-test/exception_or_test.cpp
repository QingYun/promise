#include "exception_or.h"
#include <gtest/gtest.h>

using namespace promise::_;

TEST(ExceptionOrTest, Empty) {
  ExceptionOr<int> eoi{};
  EXPECT_FALSE(eoi.isException());
  EXPECT_FALSE(eoi.isValue());
  EXPECT_TRUE(eoi.isEmpty());
}

TEST(ExceptionOrTest, SaveException) {
  {
    ExceptionOr<int> eoi{
        std::make_exception_ptr(std::runtime_error("exception"))};
    EXPECT_TRUE(eoi.isException());
    EXPECT_FALSE(eoi.isValue());
    EXPECT_FALSE(eoi.isEmpty());
    try {
      std::rethrow_exception(eoi.getException());
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("exception", e.what());
      EXPECT_TRUE(eoi.isEmpty());
    }
  }

  {
    ExceptionOr<int> eoi{};
    EXPECT_TRUE(eoi.isEmpty());
    eoi.setException(std::make_exception_ptr(std::runtime_error("exception")));
    EXPECT_TRUE(eoi.isException());
    EXPECT_FALSE(eoi.isValue());
    EXPECT_FALSE(eoi.isEmpty());
    try {
      std::rethrow_exception(eoi.getException());
    }
    catch (const std::runtime_error& e) {
      EXPECT_STREQ("exception", e.what());
      EXPECT_TRUE(eoi.isEmpty());
    }
  }
}

TEST(ExceptionOrTest, SaveScalarValue) {
  {
    ExceptionOr<int> eoi{123};
    EXPECT_TRUE(eoi.isValue());
    EXPECT_FALSE(eoi.isException());
    EXPECT_FALSE(eoi.isEmpty());
    EXPECT_EQ(123, eoi.getValue());
    EXPECT_TRUE(eoi.isEmpty());
  }

  {
    ExceptionOr<int> eoi{};
    EXPECT_TRUE(eoi.isEmpty());
    eoi.setValue(123);
    EXPECT_TRUE(eoi.isValue());
    EXPECT_FALSE(eoi.isException());
    EXPECT_FALSE(eoi.isEmpty());
    EXPECT_EQ(123, eoi.getValue());
    EXPECT_TRUE(eoi.isEmpty());
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
    EXPECT_TRUE(eoc.isValue());
    EXPECT_FALSE(eoc.isException());
    C c = eoc.getValue();
    EXPECT_EQ(123, c.value_);
    EXPECT_TRUE(eoc.isEmpty());
  }

  {
    ExceptionOr<C> eoc{};
    EXPECT_TRUE(eoc.isEmpty());
    eoc.setValue(C{ 123 });
    EXPECT_TRUE(eoc.isValue());
    EXPECT_FALSE(eoc.isException());
    EXPECT_FALSE(eoc.isEmpty());
    C c = eoc.getValue();
    EXPECT_EQ(123, c.value_);
    EXPECT_TRUE(eoc.isEmpty());
  }
}

TEST(ExceptionOrTest, DeconstructContent) {
  struct C {
    int* value_;
    C(int* value) : value_(value) {}
    C(C&& other) : value_(other.value_) { other.value_ = nullptr; }
    ~C() {
      if (value_) (*value_)--;
    }
  };

  int value = 11;
  {
    ExceptionOr<C> eoc{ C{&value} };
  }
  EXPECT_EQ(10, value);
}