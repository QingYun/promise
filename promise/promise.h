#pragma once
#include "exception_or.h"
#include <memory>
#include <tuple>
#include <type_traits>

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
  explicit Promise(F&& resolver, EventLoop& event_loop = EventLoop::instance);

  PROMISE_DISALLOW_COPY(Promise);

  Promise(Promise &&);
  Promise &operator=(Promise &&);

  Fulfiller getFulfiller();

  T wait();

  template <typename F, typename E, typename P>
  _::PromiseForResult<F, Category, T> then(F&& on_fulfill, E&& on_reject,
                                           P&& on_progress);
};

template <typename T, typename E = NoEventLoop, typename TM = SingleThreaded>
using Fulfiller = typename Promise<T, E, TM>::Fulfiller;

template <typename T, typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<std::decay_t<T>, E, TM> makePromise(T&& t);
template <typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<void, E, TM> makePromise();
template <typename T = void, typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<T, E, TM> makePromise(std::exception_ptr e);

}  // namespace promise

#include "promise-inl.h"