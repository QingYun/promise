#pragma once
#include "exception_or.h"
#include <memory>
#include <tuple>

namespace promise {

namespace _ {

template <typename T, typename EventLoop, typename ThreadingModel>
class PromiseNode;

}  // namespace _

class SingleThreaded;
class MultiThreaded;

class NoEventLoop;
class MultiThreadEventLoop;

template <typename T, typename EventLoop = NoEventLoop,
          typename ThreadingModel = SingleThreaded>
class Promise {
 public:
  class Fulfiller;

 private:
  using NodeType = _::PromiseNode<T, EventLoop, ThreadingModel>;
  std::unique_ptr<NodeType> node_;

 public:
  explicit Promise(EventLoop& event_loop = EventLoop::instance);
  template <typename F>
  Promise(F&& resolver, EventLoop& event_loop = EventLoop::instance);

  PROMISE_DISALLOW_COPY(Promise);

  Promise(Promise &&);
  Promise &operator=(Promise &&);

  Fulfiller getFulfiller();

  T wait();
};

template <typename T, typename E = NoEventLoop, typename TM = SingleThreaded>
using Fulfiller = typename Promise<T, E, TM>::Fulfiller;




}  // namespace promise

#include "promise-inl.h"