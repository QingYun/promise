#pragma once
#include "exception_or.h"
#include <memory>
#include <tuple>

namespace promise {

namespace _ {

template <typename T, typename EventLoop, typename ThreadingModel>
class PromiseNode;

template <typename PrevResult, typename OnFulfill, typename OnReject,
          typename OnProgress, typename... Attachments>
struct ContinuationTypeTrait;

template <typename F, typename... Args>
class PartiallyApplied;

}  // namespace _

class SingleThreaded;
class MultiThreaded;

class NoEventLoop;
class MultiThreadEventLoop;

using default_on_fulfill_t = std::nullptr_t;
using default_on_reject_t = std::nullptr_t;
using default_on_progress_t = std::nullptr_t;

template <typename T, typename EventLoop = NoEventLoop,
          typename ThreadingModel = SingleThreaded>
class Promise {
 public:
  class Fulfiller;

 private:
  using NodeType = _::PromiseNode<T, EventLoop, ThreadingModel>;
  std::unique_ptr<NodeType> node_;

  template <typename U>
  using Category = Promise<U, EventLoop, ThreadingModel>;

 public:
  explicit Promise(EventLoop& event_loop = EventLoop::instance);
  template <typename F>
  explicit Promise(F&& resolver, EventLoop& event_loop = EventLoop::instance);

  PROMISE_DISALLOW_COPY(Promise);

  Promise(Promise &&);
  Promise &operator=(Promise &&);

  Fulfiller getFulfiller();

  T wait();

  template <typename OnFulfill, typename OnReject, typename OnProgress,
            typename... Attachments>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfill, OnReject, OnProgress,
                               Attachments...>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<
          T, OnFulfill, OnReject, OnProgress, Attachments...>::ResultType>>
      then(OnFulfill&& on_fulfill, OnReject&& on_reject,
           OnProgress&& on_progress, Attachments&&... attachments);

  template <typename OnFulfill>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfill, std::nullptr_t,
                               std::nullptr_t>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<T, OnFulfill, std::nullptr_t,
                                                 std::nullptr_t>::ResultType>>
      then(OnFulfill&& on_fulfill);

  template <typename OnFulfill, typename OnReject>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfill, OnReject,
                               std::nullptr_t>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<T, OnFulfill, OnReject,
                                                 std::nullptr_t>::ResultType>>
      then(OnFulfill&& on_fulfill, OnReject&& on_reject);

private:
  explicit Promise(std::unique_ptr<NodeType> node);

  template <typename, typename, typename>
  friend class Promise;
  
  template <typename, typename, typename, typename>
  friend class AttachablePromiseNode;
};

template <typename T, typename E = NoEventLoop, typename TM = SingleThreaded>
using Fulfiller = typename Promise<T, E, TM>::Fulfiller;

template <typename T, typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<std::decay_t<T>, E, TM> makePromise(T&& t);
template <typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<void, E, TM> makePromise();
template <typename T = void, typename E = NoEventLoop, typename TM = SingleThreaded>
Promise<T, E, TM> makePromise(std::exception_ptr e);

template <typename F, typename... Args>
_::PartiallyApplied<F, Args...> partiallyApply(F&& f, Args&&... args);

}  // namespace promise

#include "promise-inl.h"