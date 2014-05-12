#pragma once
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace promise {

namespace _ {

//----------ObjectifyVoid----------

class Void {};
template <typename T>
struct ObjectifyVoid_ {
  using Type = T;
};
template <>
struct ObjectifyVoid_<void> {
  using Type = Void;
};
template <typename T>
using ObjectifyVoid = typename ObjectifyVoid_<T>::Type;

template <typename T>
T&& returnMaybeVoid(T&& v) {
  return std::forward<T>(v);
}

void returnMaybeVoid(Void&&) {}

//----------PromiseNode----------

class Attachment {
 public:
  virtual ~Attachment() {}

  template <typename T>
  T& As() {
    return *reinterpret_cast<T*>(this);
  }
};

template <typename T, typename EventLoop, typename ThreadingModel>
class PromiseNode {
 private:
  using ResultType = ExceptionOr<ObjectifyVoid<T>>;
  ResultType result_;

  using OnReadyCallbackType = std::function<void(PromiseNode*)>;
  OnReadyCallbackType on_ready_callback_;

  EventLoop& event_loop_;
  std::unique_ptr<Attachment> attachment_;
  ThreadingModel threading_model_;

 public:
  PromiseNode(EventLoop& event_loop, Attachment* attachment)
      : event_loop_{event_loop}, attachment_{attachment} {}

  PROMISE_DISALLOW_COPY(PromiseNode);
  PROMISE_DISALLOW_MOVE(PromiseNode);

  template <typename... U>
  void setValue(U&&... v) {
    if (!getResult().isEmpty()) return;

    threading_model_.run([&]() {
      if (!result_.isEmpty()) return;
      getResult().setValue(std::forward<U>(v)...);
      if (on_ready_callback_) on_ready_callback_(this);
    });
  }

  void setException(std::exception_ptr e) {
    if (!getResult().isEmpty()) return;

    threading_model_.run([&]() {
      if (!result_.isEmpty()) return;
      getResult().setException(std::move(e));
      if (on_ready_callback_) on_ready_callback_(this);
    });
  }

  template <typename F>
  void setOnReadyCallback(F&& f) {
    if (on_ready_callback_) return;

    threading_model_.run([&]() {
      if (on_ready_callback_) return;
      on_ready_callback_ = std::forward<F>(f);
      if (!getResult().isEmpty())
        getEventLoop().dispatch([this]() { on_ready_callback_(this); });
    });
  }

  EventLoop& getEventLoop() { return event_loop_; }
  const EventLoop& getEventLoop() const { return event_loop_; }

  ResultType& getResult() { return result_; }
  const ResultType& getResult() const { return result_; }
};

}  // namespace _

//----------SingleThreaded----------

class SingleThreaded {
public:
  template <typename F>
  void run(F&& f) {
    f();
  }
};

//----------MultiThreaded----------

class MultiThreaded {
  std::mutex lock_;

public:
  template <typename F>
  void run(F&& f) {
    std::lock_guard<std::mutex> l(lock_);
    f();
  }
};

//----------NoEventLoop----------

class NoEventLoop {
 public:
  static NoEventLoop instance;

 public:
  template <typename F>
  void dispatch(F&& f) {
    f();
  }

  void runOne() { std::this_thread::yield(); }
};

NoEventLoop NoEventLoop::instance;

//----------MultiThreadEventLoop----------

class MultiThreadEventLoop {
  std::queue<std::function<void()>> queue_;
  std::mutex lock_;
  std::condition_variable cv_;

 public:
  static MultiThreadEventLoop instance;

 public:
  template <typename F>
  void dispatch(F&& f) {
    {
      std::lock_guard<std::mutex> l(lock_);
      queue_.emplace(std::forward<F>(f));
    }
    cv_.notify_one();
  }

  void runOne() {
    std::unique_lock<std::mutex> l(lock_);
    cv_.wait(l, [this]() { return !queue_.empty(); });
    queue_.front()();
    queue_.pop();
  }
};

MultiThreadEventLoop MultiThreadEventLoop::instance;

//----------makePromise Implementation----------

template <typename T, typename E, typename TM>
Promise<std::decay_t<T>, E, TM> makePromise(T&& t) {
  Promise<std::decay_t<T>, E, TM> result;
  result.getFulfiller().fulfill(std::forward<T>(t));
  return result;
}

template <typename E, typename TM>
Promise<void, E, TM> makePromise() {
  Promise<void, E, TM> result;
  result.getFulfiller().fulfill();
  return result;
}

template <typename T, typename E, typename TM>
Promise<T, E, TM> makePromise(std::exception_ptr e) {
  Promise<T, E, TM> result;
  result.getFulfiller().reject(std::move(e));
  return result;
}

//----------Promise Implementation----------

#define PROMISE_TEMPLATE_LIST \
  template <typename T, typename EventLoop, typename ThreadingModel>
#define PROMISE Promise<T, EventLoop, ThreadingModel>

PROMISE_TEMPLATE_LIST
class PROMISE::Fulfiller {
  using PromiseType = PROMISE;
  using NodeType = typename PromiseType::NodeType;
  NodeType* node_;

 public:
  Fulfiller() : node_{nullptr} {}
  Fulfiller(NodeType* node) : node_{node} {}

  Fulfiller(Fulfiller&& other) : node_{other.node_} {
    other.node_ = nullptr;
  }
  Fulfiller& operator= (Fulfiller&& other) {
    node_ = other.node_;
    other.node_ = nullptr;
    return *this;
  }

  ~Fulfiller() {
    if (node_)
      reject<std::logic_error>("~Fulfiller");
  }

  template <typename... U>
  void fulfill(U&&... v) {
    node_->setValue(std::forward<U>(v)...);
  }

  template <typename E, typename... T>
  void reject(T&&... t) {
    node_->setException(std::make_exception_ptr(E{std::forward<T>(t)...}));
  }
  void reject(std::exception_ptr e) { node_->setException(std::move(e)); }
};

PROMISE_TEMPLATE_LIST
PROMISE::Promise(EventLoop& event_loop)
    : node_{new _::PromiseNode<T, EventLoop, ThreadingModel>{event_loop, nullptr}} {}

PROMISE_TEMPLATE_LIST
template <typename F>
PROMISE::Promise(F&& f, EventLoop& event_loop)
    : Promise{event_loop} {
  f(getFulfiller());
}

PROMISE_TEMPLATE_LIST
PROMISE::Promise(Promise&& other) : node_{std::move(other.node_)} {}

PROMISE_TEMPLATE_LIST
PROMISE& PROMISE::operator=(Promise&& other) {
  node_ = std::move(other.node_);
  return *this;
}

PROMISE_TEMPLATE_LIST
typename PROMISE::Fulfiller PROMISE::getFulfiller() {
  return Fulfiller{node_.get()};
}

PROMISE_TEMPLATE_LIST
T PROMISE::wait() {
  bool finish = false;
  node_->setOnReadyCallback([&finish](NodeType*) mutable {
    finish = true;
  });

  while (!finish)
    node_->getEventLoop().runOne();

  auto& result = node_->getResult();
  assert(!result.isEmpty());

  if (result.isException())
    std::rethrow_exception(result.getException());

  return _::returnMaybeVoid(std::move(result.getValue()));
}

#undef PROMISE_TEMPLATE_LIST
#undef PROMISE

}  // namespace promise