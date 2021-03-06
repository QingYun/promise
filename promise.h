#pragma once
#include <memory>
#include <tuple>
#include <type_traits>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#define PROMISE_DISALLOW_COPY(class_name) \
  class_name(const class_name&) = delete; \
  class_name& operator=(const class_name&) = delete

#define PROMISE_DISALLOW_MOVE(class_name) \
  class_name(class_name&&) = delete;      \
  class_name& operator=(class_name&&) = delete

#ifdef NDEBUG

#define PROMISE_ASSERT(expr) ((void)0)

#else

#define PROMISE_ASSERT(expr) do { if (!(expr)) { throw std::runtime_error{#expr "\n" __FILE__}; } } while (0)

#endif

namespace promise {

namespace _ {

template <typename T, typename EventLoop, typename ThreadingModel>
class PromiseNode;

template <typename PrevResult, typename OnFulfilled, typename OnRejected,
          typename OnProgress, typename... Attachments>
struct ContinuationTypeTrait;

template <typename F, typename... Args>
class PartiallyApplied;

}  // namespace _

class SingleThreaded;
class MultiThreaded;

class NoEventLoop;
class MultiThreadEventLoop;

using default_on_fulfilled_t = std::nullptr_t;
using default_on_rejected_t = std::nullptr_t;
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
  explicit Promise(EventLoop& event_loop = EventLoop::getInstance());
  template <typename F>
  explicit Promise(F&& resolver, EventLoop& event_loop = EventLoop::getInstance());

  PROMISE_DISALLOW_COPY(Promise);

  Promise(Promise&&);
  Promise& operator=(Promise&&);

  Fulfiller getFulfiller();

  T wait();

  template <typename OnFulfilled, typename OnRejected, typename OnProgress,
            typename... Attachments>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfilled, OnRejected, OnProgress,
                               Attachments...>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<
          T, OnFulfilled, OnRejected, OnProgress, Attachments...>::ResultType>>
      then(OnFulfilled&& on_fulfilled, OnRejected&& on_rejected,
           OnProgress&& on_progress, Attachments&&... attachments);

  template <typename OnFulfilled>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfilled, std::nullptr_t,
                               std::nullptr_t>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<T, OnFulfilled, std::nullptr_t,
                                                 std::nullptr_t>::ResultType>>
      then(OnFulfilled&& on_fulfilled);

  template <typename OnFulfilled, typename OnRejected>
  std::enable_if_t<
      _::ContinuationTypeTrait<T, OnFulfilled, OnRejected,
                               std::nullptr_t>::is_valid_continuation,
      Category<typename _::ContinuationTypeTrait<T, OnFulfilled, OnRejected,
                                                 std::nullptr_t>::ResultType>>
      then(OnFulfilled&& on_fulfilled, OnRejected&& on_rejected);

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
template <typename T = void, typename E = NoEventLoop,
          typename TM = SingleThreaded>
Promise<T, E, TM> makePromise(std::exception_ptr e);

template <typename F, typename... Args>
_::PartiallyApplied<F, Args...> partiallyApply(F&& f, Args&&... args);

namespace _ {

//----------ObjectifyVoid----------

struct Void {
  Void() {}
  Void(const Void&) {}
};

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

inline void returnMaybeVoid(Void&&) {}

template <
    typename F, typename... Args,
    std::enable_if_t<
        !std::is_same<void, std::result_of_t<F(Args&&...)>>::value>* = nullptr>
auto objectifyReturnedVoid(F&& f, Args&&... args)
    -> std::result_of_t<F(Args&&...)> {
  return f(std::forward<Args>(args)...);
}

template <typename F, typename... Args,
          std::enable_if_t<std::is_same<
              void, std::result_of_t<F(Args&&...)>>::value>* = nullptr>
Void objectifyReturnedVoid(F&& f, Args&&... args) {
  f(std::forward<Args>(args)...);
  return Void{};
}

//----------ExceptionOr----------

template <
    typename T,
    bool TIsLarger = (sizeof(typename std::aligned_storage<
                          sizeof(T), std::alignment_of<T>::value>::type) >
                      sizeof(typename std::aligned_storage<
                          sizeof(std::exception_ptr),
                          std::alignment_of<std::exception_ptr>::value>::type))>
struct Storage_;

template <typename T>
struct Storage_<T, true> {
  using Type = typename std::aligned_storage<sizeof(T),
                                             std::alignment_of<T>::value>::type;
};

template <typename T>
struct Storage_<T, false> {
  using Type = typename std::aligned_storage<
      sizeof(std::exception_ptr),
      std::alignment_of<std::exception_ptr>::value>::type;
};

template <typename T>
class Storage {
  typename Storage_<T>::Type storage_;

 public:
  template <typename T>
  T* As() {
    return reinterpret_cast<T*>(&storage_);
  }
};

template <typename T>
class ExceptionOr {
  Storage<T> storage_;

  enum class State { Empty, Exception, Value } state_;

 public:
  ExceptionOr() : state_{State::Empty} {}
  ExceptionOr(T&& value) : ExceptionOr{} { setValue(std::move(value)); }
  ExceptionOr(std::exception_ptr exception) : ExceptionOr{} {
    setException(std::move(exception));
  }

  ~ExceptionOr() {
    using namespace std;
    if (isException())
      storage_.As<exception_ptr>()->~exception_ptr();
    else if (isValue())
      storage_.As<T>()->~T();
  }

  PROMISE_DISALLOW_COPY(ExceptionOr);
  PROMISE_DISALLOW_MOVE(ExceptionOr);

  bool isEmpty() const { return state_ == State::Empty; }
  bool isException() const { return state_ == State::Exception; }
  bool isValue() const { return state_ == State::Value; }

  void setException(std::exception_ptr exception) {
    PROMISE_ASSERT(isEmpty());
    new (storage_.As<void>()) std::exception_ptr{std::move(exception)};
    state_ = State::Exception;
  }
  template <typename... U>
  void setValue(U&&... value) {
    PROMISE_ASSERT(isEmpty());
    new (storage_.As<void>()) T{std::forward<U>(value)...};
    state_ = State::Value;
  }

  std::exception_ptr getException() {
    state_ = State::Empty;
    return std::move(*storage_.As<std::exception_ptr>());
  }
  T getValue() {
    state_ = State::Empty;
    return std::move(*storage_.As<T>());
  }
};

//----------makeCopyableFunctor----------
template <typename F>
class FunctorCopyableWrapper {
  mutable std::decay_t<F> func_;

 public:
  explicit FunctorCopyableWrapper(F&& f) : func_{std::move(f)} {}

  FunctorCopyableWrapper(const FunctorCopyableWrapper& other)
      : func_{std::move(other.func_)} {}
  FunctorCopyableWrapper& operator=(const FunctorCopyableWrapper& other) {
    func_ = std::move(other.func_);
    return *this;
  }

  FunctorCopyableWrapper(FunctorCopyableWrapper&& other)
      : func_{std::move(other.func_)} {}
  FunctorCopyableWrapper& operator=(FunctorCopyableWrapper&& other) {
    func_ = std::move(other.func_);
    return *this;
  }

  template <typename... Args>
  auto operator()(Args&&... args)
      -> decltype(func_(std::forward<Args>(args)...)) {
    return func_(std::forward<Args>(args)...);
  }
};

template <typename F, typename std::enable_if_t<
                          !std::is_copy_assignable<F>::value>* = nullptr>
FunctorCopyableWrapper<F> makeCopyableFunctor_(F&& f) {
  return FunctorCopyableWrapper<F>{std::forward<F>(f)};
}

template <typename F, typename std::enable_if_t<
                          std::is_copy_assignable<F>::value>* = nullptr>
F&& makeCopyableFunctor_(F&& f) {
  return std::forward<F>(f);
}

class __IsCopyableTest {
 public:
  __IsCopyableTest(const __IsCopyableTest&) = delete;
  __IsCopyableTest& operator=(const __IsCopyableTest&) = delete;

  __IsCopyableTest(__IsCopyableTest&&) {}
  __IsCopyableTest& operator=(__IsCopyableTest&&) { return *this; }
};

template <
    typename F,
    std::enable_if_t<std::is_copy_assignable<__IsCopyableTest>::value == false,
                     F>* = nullptr>
auto makeCopyableFunctor(F&& f)
    -> decltype(makeCopyableFunctor_(std::forward<F>(f))) {
  return makeCopyableFunctor_(std::forward<F>(f));
}

template <
    typename F,
    std::enable_if_t<std::is_copy_assignable<__IsCopyableTest>::value == true,
                     F>* = nullptr>
FunctorCopyableWrapper<F> makeCopyableFunctor(F&& f) {
  return FunctorCopyableWrapper<F>{std::forward<F>(f)};
}

//----------IndexList----------

template <std::size_t...>
struct Indexes {};

template <typename Idx, std::size_t N, std::size_t Max, bool End>
struct IndexList_;

template <std::size_t... Idx, std::size_t N, std::size_t Max>
struct IndexList_<Indexes<Idx...>, N, Max,
                  false> : public IndexList_<Indexes<Idx..., N>, N + 1, Max,
                                             N + 1 == Max> {};

template <std::size_t... Idx, std::size_t Max>
struct IndexList_<Indexes<Idx...>, Max, Max, true> {
  using Type = Indexes<Idx..., Max>;
};

template <std::size_t Min, std::size_t Max>
using IndexList = typename IndexList_<Indexes<>, Min, Max, Min == Max>::Type;

//----------forwardMaybeNull----------

template <typename T, typename U>
struct ForwarderMaybeNull {
  using ResultType = T&&;
  static ResultType go(T&& arg, U&&) { return static_cast<T&&>(arg); }
};

template <typename U>
struct ForwarderMaybeNull<std::nullptr_t, U> {
  using ResultType = U&&;
  static ResultType go(std::nullptr_t, U&& rvalue_default) {
    return std::move(rvalue_default);
  }
};

template <typename T, typename U>
typename ForwarderMaybeNull<T, U>::ResultType forwardMaybeNull(T&& t, U&& u) {
  return ForwarderMaybeNull<T, U>::go(std::forward<T>(t), std::forward<U>(u));
}

//----------callContinuation----------

template <typename F>
struct IsNullaryFunction {
 private:
  template <typename F>
  static std::true_type test(decltype(std::declval<F>()())*);
  template <typename>
  static std::false_type test(...);

 public:
  static const bool value = decltype(test<F>(nullptr))::value;
};

template <typename F, typename P1>
struct IsUnaryFunction {
 private:
  template <typename F>
  static std::true_type test(decltype(std::declval<F>()(std::declval<P1>()))*);
  template <typename>
  static std::false_type test(...);

 public:
  static const bool value = decltype(test<F>(nullptr))::value;
};

template <typename F, typename P1, typename P2>
struct IsBinaryFunction {
 private:
  template <typename F>
  static std::true_type test(decltype(std::declval<F>()(std::declval<P1>(),
                                                        std::declval<P2>()))*);
  template <typename>
  static std::false_type test(...);

 public:
  static const bool value = decltype(test<F>(nullptr))::value;
};

template <typename F, typename PrevResult, typename Attachment,
          bool enable = IsUnaryFunction<F, PrevResult>::value>
struct CallContinuation;

template <typename F, typename PrevResult, typename Attachment>
struct CallContinuation<F, PrevResult, Attachment, true> {
  using ResultType = ObjectifyVoid<std::result_of_t<F(PrevResult)>>;

  template <typename F, typename PrevResult, typename Attachment>
  static ResultType go(F&& f, PrevResult&& prev_result, Attachment&) {
    return objectifyReturnedVoid(std::forward<F>(f),
                                 std::forward<PrevResult>(prev_result));
  }
};

template <typename F, typename PrevResult, typename Attachment>
struct CallContinuation<F, PrevResult, Attachment, false> {
  static_assert(IsBinaryFunction<F, PrevResult, Attachment>::value, "");

  using ResultType = ObjectifyVoid<std::result_of_t<F(PrevResult, Attachment)>>;

  template <typename F, typename PrevResult, typename Attachment>
  static ResultType go(F&& f, PrevResult&& prev_result,
                       Attachment& attachment) {
    return objectifyReturnedVoid(
        std::forward<F>(f), std::forward<PrevResult>(prev_result), attachment);
  }
};

template <typename F, typename Attachment,
          bool acceptVoid = IsUnaryFunction<F, Void>::value,
          bool acceptAttachment = IsUnaryFunction<F, Attachment>::value>
struct CallVoidContinuation;

template <typename F, typename Attachment>
struct CallVoidContinuation<F, Attachment, false, true> {
  using RawResult = std::result_of_t<F(Attachment)>;
  using ResultType = ObjectifyVoid<RawResult>;

  template <typename F, typename Attachment>
  static ResultType go(F&& f, Void, Attachment& attachment) {
    return objectifyReturnedVoid(std::forward<F>(f), attachment);
  }
};

template <typename F, typename Attachment>
struct CallVoidContinuation<F, Attachment, true, false> {
  using RawResult = std::result_of_t<F(Void)>;
  using ResultType = ObjectifyVoid<RawResult>;

  template <typename F, typename Attachment>
  static ResultType go(F&& f, Void, Attachment&) {
    return objectifyReturnedVoid(std::forward<F>(f), Void{});
  }
};

template <typename F, typename Attachment>
struct CallVoidContinuation<F, Attachment, false, false> {
  static_assert(IsNullaryFunction<F>::value, "");

  using RawResult = std::result_of_t<F()>;
  using ResultType = ObjectifyVoid<RawResult>;

  template <typename F, typename Attachment>
  static ResultType go(F&& f, Void, Attachment&) {
    return objectifyReturnedVoid(std::forward<F>(f));
  }
};

template <typename F, typename Attachment>
struct CallContinuation<F, Void, Attachment,
                        true> : public CallVoidContinuation<F, Attachment> {};

template <typename F, typename Attachment>
struct CallContinuation<F, Void, Attachment,
                        false> : public CallVoidContinuation<F, Attachment> {};

template <typename F, typename PrevResult, typename Attachment>
typename CallContinuation<F, PrevResult, Attachment&>::ResultType
callContinuation(F&& f, PrevResult&& prev_result, Attachment& attachment) {
  return CallContinuation<F, PrevResult, Attachment&>::go(
      std::forward<F>(f), std::forward<PrevResult>(prev_result), attachment);
}

//----------PromiseNode----------
class PromiseNodeBase {
 public:
  virtual ~PromiseNodeBase() {}
};

template <typename T, typename EventLoop, typename ThreadingModel>
class PromiseNode : public PromiseNodeBase {
 private:
  using Result = ObjectifyVoid<T>;
  ExceptionOr<Result> result_;

  using OnReadyCallback = std::function<void(PromiseNode*)>;
  OnReadyCallback on_ready_callback_;
  using OnProgressCallback = std::function<void(PromiseNode*, Result)>;
  OnProgressCallback on_progress_callback_;

  EventLoop& event_loop_;
  std::unique_ptr<PromiseNodeBase> prev_;
  ThreadingModel threading_model_;

 public:
  PromiseNode(EventLoop& event_loop, std::unique_ptr<PromiseNodeBase> prev =
                                         std::unique_ptr<PromiseNodeBase>{})
      : event_loop_{event_loop}, prev_{std::move(prev)} {}

  PROMISE_DISALLOW_COPY(PromiseNode);
  PROMISE_DISALLOW_MOVE(PromiseNode);

  template <typename... U>
  void setValue(U&&... v) {
    {
      std::lock_guard<typename ThreadingModel::ResultLock> l{
          threading_model_.getResultLock()};
      if (getResult().isEmpty()) getResult().setValue(std::forward<U>(v)...);
    }

    if (getOnReadySafe())
      getEventLoop().dispatch([this]() {
      PROMISE_ASSERT(on_ready_callback_);
        on_ready_callback_(this);
      });
  }

  void setException(std::exception_ptr e) {
    {
      std::lock_guard<typename ThreadingModel::ResultLock> l{
          threading_model_.getResultLock()};
      if (getResult().isEmpty()) getResult().setException(std::move(e));
    }

    if (getOnReadySafe())
      getEventLoop().dispatch([this]() {
      PROMISE_ASSERT(on_ready_callback_);
        on_ready_callback_(this);
      });
  }

  template <typename F>
  void setOnReadyCallback(F&& f) {
    setOnReadySafe(std::forward<F>(f));
    if (!getResultSafe().isEmpty())
      getEventLoop().dispatch([this]() {
      PROMISE_ASSERT(on_ready_callback_);
        on_ready_callback_(this);
      });
  }

  template <typename F>
  void setOnProgressCallback(F&& f) {
    setOnProgressSafe(std::forward<F>(f));
  }

  template <typename... U>
  void onProgress(U&&... u) {
    if (getOnProgressSafe())
      getEventLoop().dispatch(partiallyApply([this](Result r) {
                                               PROMISE_ASSERT(
                                                   on_progress_callback_);
                                               on_progress_callback_(
                                                   this, std::move(r));
                                             },
                                             Result{std::forward<U>(u)...}));
  }

  EventLoop& getEventLoop() { return event_loop_; }
  const EventLoop& getEventLoop() const { return event_loop_; }

  ExceptionOr<Result>& getResult() { return result_; }
  const ExceptionOr<Result>& getResult() const { return result_; }

  template <std::size_t>
  static std::tuple<>& getAttachment() {
    static std::tuple<> dummy;
    return dummy;
  }

 private:
  ExceptionOr<Result>& getResultSafe() {
    std::lock_guard<typename ThreadingModel::ResultLock> l{
        threading_model_.getResultLock()};
    return result_;
  }

  template <typename F>
  void setOnReadySafe(F&& f) {
    std::lock_guard<typename ThreadingModel::OnReadyLock> l{
        threading_model_.getOnReadyLock()};
    if (!on_ready_callback_)
      on_ready_callback_ = makeCopyableFunctor(std::forward<F>(f));
  }

  OnReadyCallback& getOnReadySafe() {
    std::lock_guard<typename ThreadingModel::OnReadyLock> l{
        threading_model_.getOnReadyLock()};
    return on_ready_callback_;
  }

  template <typename F>
  void setOnProgressSafe(F&& f) {
    std::lock_guard<typename ThreadingModel::OnProgressLock> l{
        threading_model_.getOnProgressLock()};
    if (!on_progress_callback_)
      on_progress_callback_ = makeCopyableFunctor(std::forward<F>(f));
  }

  OnProgressCallback& getOnProgressSafe() {
    std::lock_guard<typename ThreadingModel::OnProgressLock> l{
        threading_model_.getOnProgressLock()};
    return on_progress_callback_;
  }
};

template <typename T, typename EventLoop, typename ThreadingModel,
          typename... Attachments>
class AttachablePromiseNode : public PromiseNode<T, EventLoop, ThreadingModel> {
  std::tuple<std::decay_t<Attachments>...> attachments_;

 public:
  template <typename... As>
  AttachablePromiseNode(EventLoop& event_loop,
                        std::unique_ptr<PromiseNodeBase> prev,
                        As&&... attachments)
      : PromiseNode{event_loop, std::move(prev)},
        attachments_{std::forward<As>(attachments)...} {}

  template <std::size_t N>
  typename std::tuple_element<N, decltype(attachments_)>::type&
  getAttachment() {
    return std::get<N>(attachments_);
  }
};

//----------ContinuationTypeTrait----------

template <typename F, typename P, bool Enable = IsUnaryFunction<F, P>::value>
struct UnaryFunctionResult;

template <typename F, typename P>
struct UnaryFunctionResult<F, P, true> {
  using Type = std::result_of_t<F(P)>;
};

template <typename F, typename P>
struct UnaryFunctionResult<F, P, false> {};

template <typename F, typename P1, typename P2,
          bool Enable = IsBinaryFunction<F, P1, P2>::value>
struct BinaryFunctionResult;

template <typename F, typename P1, typename P2>
struct BinaryFunctionResult<F, P1, P2, true> {
  using Type = std::result_of_t<F(P1, P2)>;
};

template <typename F, typename P1, typename P2>
struct BinaryFunctionResult<F, P1, P2, false> {};

template <typename F, typename P1, typename P2>
struct FunctionResultType_ : public UnaryFunctionResult<F, P1>,
                             public BinaryFunctionResult<F, P1, P2> {};

template <typename F, typename P>
struct FunctionResultType_<F, void, P> {
  using Type = typename CallVoidContinuation<F, P>::RawResult;
};

template <typename F, typename P1, typename P2>
using FunctionResultType = typename FunctionResultType_<F, P1, P2>::Type;

template <typename T, typename Attachment>
struct AttachableNodeMaker_ {
  template <typename E, typename Th>
  struct NodeMaker {
    using Type = AttachablePromiseNode<T, E, Th, Attachment>;

    static std::unique_ptr<Type> make(E& event_loop,
                                      std::unique_ptr<PromiseNodeBase> prev,
                                      Attachment&& attachment) {
      return std::unique_ptr<Type>{
          new Type{event_loop, std::move(prev), std::move(attachment)}};
    }
  };
};

template <typename T, typename E, typename Th, typename Attachment>
struct AttachableNodeMaker_<Promise<T, E, Th>, Attachment> {
  template <typename E, typename Th>
  struct NodeMaker {
    using Type = AttachablePromiseNode<T, E, Th, Attachment, Promise<T, E, Th>>;

    static std::unique_ptr<Type> make(E& event_loop,
                                      std::unique_ptr<PromiseNodeBase> prev,
                                      Attachment&& attachment) {
      return std::unique_ptr<Type>{
          new Type{event_loop, std::move(prev), std::move(attachment),
                   std::unique_ptr<PromiseNode<T, E, Th>>{}}};
    }
  };
};

template <typename T, typename Attachment>
struct NodeMaker_ : public AttachableNodeMaker_<T, Attachment> {};

template <typename T>
struct NodeMaker_<T, std::tuple<>> {
  template <typename E, typename Th>
  struct NodeMaker {
    using Type = PromiseNode<T, E, Th>;

    static std::unique_ptr<Type> make(E& event_loop,
                                      std::unique_ptr<PromiseNodeBase> prev,
                                      std::tuple<>&& attachment) {
      return std::unique_ptr<Type>{new Type{event_loop, std::move(prev)}};
    }
  };
};

template <typename T, typename Attachment>
struct JoinPromise : public NodeMaker_<T, Attachment> {
  using ResultType = T;
};

template <typename T, typename E, typename Th, typename Attachment>
struct JoinPromise<Promise<T, E, Th>, Attachment> : public AttachableNodeMaker_<
                                                        Promise<T, E, Th>,
                                                        Attachment> {
  using ResultType = T;
};

template <typename PrevResult, typename OnFulfilled, typename OnRejected,
          typename Attachment>
struct ContinuationResult
    : public JoinPromise<FunctionResultType<OnFulfilled, PrevResult, Attachment&>,
                         Attachment> {
  static const bool valid_result_type = std::is_same<
      ResultType,
      FunctionResultType<OnRejected, std::exception_ptr, Attachment&>>::value;
};

template <typename PrevResult, typename OnRejected, typename Attachment>
struct ContinuationResult<
    PrevResult, std::nullptr_t, OnRejected,
    Attachment> : public JoinPromise<FunctionResultType<OnRejected,
                                                        std::exception_ptr,
                                                        Attachment&>,
                                     Attachment> {
  static const bool valid_result_type = true;
};

template <typename PrevResult, typename OnFulfilled, typename Attachment>
struct ContinuationResult<
    PrevResult, OnFulfilled, std::nullptr_t,
    Attachment> : public JoinPromise<FunctionResultType<OnFulfilled, PrevResult,
                                                        Attachment&>,
                                     Attachment> {
  static const bool valid_result_type = true;
};

template <typename PrevResult, typename Attachment>
struct ContinuationResult<PrevResult, std::nullptr_t, std::nullptr_t,
                          Attachment> : public JoinPromise<PrevResult,
                                                           Attachment> {
  static const bool valid_result_type = true;
};

template <typename PrevResult, typename OnProgress, typename Attachment>
struct IsValidOnProgress {
  static const bool value =
      IsUnaryFunction<OnProgress, PrevResult>::value ||
      IsUnaryFunction<OnProgress, Attachment>::value ||
      IsBinaryFunction<OnProgress, PrevResult, Attachment>::value;
};

template <typename OnProgress, typename Attachment>
struct IsValidOnProgress<void, OnProgress, Attachment> {
  static const bool value = IsNullaryFunction<OnProgress>::value ||
                            IsUnaryFunction<OnProgress, Attachment>::value ||
                            IsUnaryFunction<OnProgress, Void>::value;
};

template <typename PrevResult, typename Attachment>
struct IsValidOnProgress<PrevResult, std::nullptr_t, Attachment> {
  static const bool value = true;
};

template <typename Attachment>
struct IsValidOnProgress<void, std::nullptr_t, Attachment> {
  static const bool value = true;
};

template <typename PrevResult, typename OnFulfilled, typename OnRejected,
          typename OnProgress, typename... Attachments>
struct ContinuationTypeTrait
    : public ContinuationResult<PrevResult, OnFulfilled, OnRejected,
                                std::tuple<Attachments...>> {
  static const bool is_valid_continuation =
      valid_result_type &&
      IsValidOnProgress<PrevResult, OnProgress,
                        std::tuple<Attachments...>&>::value;
};

//----------transformAndFulfill----------
template <typename Node, typename Continuation, typename PrevResult>
void transformAndFulfill(Node* node, Continuation&& continuation,
                         PrevResult&& prev_result) {
  try {
    node->setValue(callContinuation(std::forward<Continuation>(continuation),
                                    std::forward<PrevResult>(prev_result),
                                    node->getAttachment<0>()));
  }
  catch (...) {
    node->setException(std::current_exception());
  }
}

template <typename T, typename E, typename Th, typename Attachment,
          typename ResultPromise, typename Continuation, typename PrevResult>
void transformAndFulfill(
    AttachablePromiseNode<T, E, Th, Attachment, ResultPromise>* node,
    Continuation&& continuation, PrevResult&& prev_result) {
  try {
    ResultPromise result_promise = callContinuation(
        std::forward<Continuation>(continuation),
        std::forward<PrevResult>(prev_result), node->getAttachment<0>());
    result_promise.then([node](T t) { node->setValue(std::move(t)); },
                        [node](std::exception_ptr e) {
                          node->setException(std::move(e));
                        },
                        [node](T t) { node->onProgress(std::move(t)); });
    node->getAttachment<1>() = std::move(result_promise);
  }
  catch (...) {
    node->setException(std::current_exception());
  }
}

}  // namespace _

//----------SingleThreaded----------

namespace _ {

class DummyLock {
 public:
  void lock() {}
  void unlock() {}
};

}  // namespace _

class SingleThreaded : public _::DummyLock {
 public:
  using ResultLock = _::DummyLock;
  using OnReadyLock = _::DummyLock;
  using OnProgressLock = _::DummyLock;

  ResultLock& getResultLock() { return *this; }
  OnReadyLock& getOnReadyLock() { return *this; }
  OnProgressLock& getOnProgressLock() { return *this; }
};

//----------MultiThreaded----------

class MultiThreaded {
 public:
  using ResultLock = std::mutex;
  using OnReadyLock = std::mutex;
  using OnProgressLock = std::mutex;

 private:
  ResultLock result_lock_;
  OnReadyLock on_ready_lock_;
  OnProgressLock on_progress_lock_;

 public:
  ResultLock& getResultLock() { return result_lock_; }
  OnReadyLock& getOnReadyLock() { return on_ready_lock_; }
  OnProgressLock& getOnProgressLock() { return on_progress_lock_; }
};

//----------NoEventLoop----------

class NoEventLoop {
 public:
  template <typename F>
  void dispatch(F&& f) {
    f();
  }

  void runOne() { std::this_thread::yield(); }

  static NoEventLoop& getInstance() {
    static NoEventLoop instance;
    return instance;
  }
};

//----------MultiThreadEventLoop----------

class MultiThreadEventLoop {
  std::queue<std::function<void()>> queue_;
  std::mutex lock_;
  std::condition_variable cv_;

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

  static MultiThreadEventLoop& getInstance() {
    static MultiThreadEventLoop instance;
    return instance;
  }
};

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

//----------partiallyApply Implementation----------

namespace _ {

template <typename F, typename T, std::size_t... Idx, typename... Args>
auto tupleApply(F&& f, T&& t, Indexes<Idx...>, Args&&... args)
    -> decltype(f(std::move(std::get<Idx>(t))...,
                  std::forward<Args>(args)...)) {
  return f(std::move(std::get<Idx>(t))..., std::forward<Args>(args)...);
}

template <typename F, typename... Args>
class PartiallyApplied {
  std::decay_t<F> func_;
  std::tuple<std::decay_t<Args>...> args_;
  IndexList<0, sizeof...(Args) - 1> arg_index_list_;

 public:
  template <typename TP>
  PartiallyApplied(F&& f, TP&& t)
      : func_{std::move(f)}, args_{std::forward<TP>(t)} {}

  PROMISE_DISALLOW_COPY(PartiallyApplied);

  PartiallyApplied(PartiallyApplied&& other)
      : func_(std::move(other.func_)), args_(std::move(other.args_)) {}
  PartiallyApplied& operator=(PartiallyApplied&& other) {
    func_ = std::move(other.func_);
    args_ = std::move(other.args_);
    return *this;
  }

  template <typename... T>
  auto operator()(T&&... t)
      -> decltype(tupleApply(func_, args_, arg_index_list_,
                             std::forward<T>(t)...)) {
    return tupleApply(std::move(func_), std::move(args_), arg_index_list_,
                      std::forward<T>(t)...);
  }
};

}  // namespace _

template <typename F, typename... Args>
_::PartiallyApplied<F, Args...> partiallyApply(F&& f, Args&&... args) {
  return _::PartiallyApplied<F, Args...>{
      std::forward<F>(f), std::forward_as_tuple(std::forward<Args>(args)...)};
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
  explicit Fulfiller(NodeType* node) : node_{node} {}

  Fulfiller(Fulfiller&& other) : node_{other.node_} { other.node_ = nullptr; }
  Fulfiller& operator=(Fulfiller&& other) {
    node_ = other.node_;
    other.node_ = nullptr;
    return *this;
  }

  ~Fulfiller() {
    if (node_) reject<std::logic_error>("~Fulfiller");
  }

  template <typename... U>
  void fulfill(U&&... v) {
    PROMISE_ASSERT(node_);
    node_->setValue(std::forward<U>(v)...);
    node_ = nullptr;
  }

  template <typename E, typename... T>
  void reject(T&&... t) {
    reject(std::make_exception_ptr(E{std::forward<T>(t)...}));
  }
  void reject(std::exception_ptr e) {
    PROMISE_ASSERT(node_);
    node_->setException(std::move(e));
    node_ = nullptr;
  }

  template <typename... U>
  void progress(U&&... u) {
    PROMISE_ASSERT(node_);
    node_->onProgress(std::forward<U>(u)...);
  }
};

PROMISE_TEMPLATE_LIST
PROMISE::Promise(EventLoop& event_loop)
    : node_{new _::PromiseNode<T, EventLoop, ThreadingModel>{event_loop}} {}

PROMISE_TEMPLATE_LIST
template <typename F>
PROMISE::Promise(F&& f, EventLoop& event_loop)
    : Promise{event_loop} {
  f(getFulfiller());
}

PROMISE_TEMPLATE_LIST
PROMISE::Promise(std::unique_ptr<NodeType> node) : node_{std::move(node)} {}

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
  node_->setOnReadyCallback([&finish](NodeType*) mutable { finish = true; });

  while (!finish) node_->getEventLoop().runOne();

  auto& result = node_->getResult();
  PROMISE_ASSERT(!result.isEmpty());

  if (result.isException()) std::rethrow_exception(result.getException());

  return _::returnMaybeVoid(std::move(result.getValue()));
}

PROMISE_TEMPLATE_LIST
template <typename OnFulfilled, typename OnRejected, typename OnProgress,
          typename... Attachments>
auto PROMISE::then(OnFulfilled&& on_fulfilled, OnRejected&& on_rejected,
                   OnProgress&& on_progress, Attachments&&... attachments)
    -> std::enable_if_t<
          _::ContinuationTypeTrait<T, OnFulfilled, OnRejected, OnProgress,
                                   Attachments...>::is_valid_continuation,
          Category<typename _::ContinuationTypeTrait<
              T, OnFulfilled, OnRejected, OnProgress,
              Attachments...>::ResultType>> {
  using namespace _;
  using ResultType = typename ContinuationTypeTrait<
      T, OnFulfilled, OnRejected, OnProgress, Attachments...>::ResultType;
  using ResultPromise = Category<ResultType>;

  auto default_on_fulfilled = [](ObjectifyVoid<T> v) {
    return returnMaybeVoid(std::move(v));
  };
  using OnFulfilledType = decltype(forwardMaybeNull<OnFulfilled>(
      std::forward<OnFulfilled>(on_fulfilled), std::move(default_on_fulfilled)));

  auto default_on_rejected = [](std::exception_ptr e) {
    std::rethrow_exception(e);
    return returnMaybeVoid(
        std::move(*reinterpret_cast<ObjectifyVoid<ResultType>*>(nullptr)));
  };
  using OnRejectedType = decltype(forwardMaybeNull<OnRejected>(
      std::forward<OnRejected>(on_rejected), std::move(default_on_rejected)));

  auto default_on_progress = [](ObjectifyVoid<T>&&) {};
  using OnProgressType = decltype(forwardMaybeNull<OnProgress>(
      std::forward<OnProgress>(on_progress), std::move(default_on_progress)));

  auto old_node = node_.get();
  auto new_node = ContinuationTypeTrait<T, OnFulfilled, OnRejected, OnProgress,
                                        Attachments...>::
      NodeMaker<EventLoop, ThreadingModel>::make(
          old_node->getEventLoop(), std::move(node_),
          std::forward_as_tuple(std::forward<Attachments>(attachments)...));
  auto new_node_ptr = new_node.get();

  old_node->setOnReadyCallback(partiallyApply(
      [new_node_ptr](OnFulfilledType on_fulfilled, OnRejectedType on_rejected,
                     NodeType* node) {
        auto& result = node->getResult();
        if (result.isException())
          transformAndFulfill(new_node_ptr, on_rejected, result.getException());
        else
          transformAndFulfill(new_node_ptr, on_fulfilled, result.getValue());
      },
      forwardMaybeNull<OnFulfilled>(std::forward<OnFulfilled>(on_fulfilled),
                                  std::move(default_on_fulfilled)),
      forwardMaybeNull<OnRejected>(std::forward<OnRejected>(on_rejected),
                                 std::move(default_on_rejected))));

  old_node->setOnProgressCallback(
      partiallyApply([new_node_ptr](OnProgressType on_progress, NodeType* node,
                                    ObjectifyVoid<T> value) {
                       try {
                         callContinuation(on_progress, std::move(value),
                                          new_node_ptr->getAttachment<0>());
                       }
                       catch (...) {
                         new_node_ptr->setException(std::current_exception());
                       }
                     },
                     forwardMaybeNull<OnProgress>(
                         std::forward<OnProgress>(on_progress),
                         std::move(default_on_progress))));

  return ResultPromise{
      std::unique_ptr<ResultPromise::NodeType>{std::move(new_node)}};
}

PROMISE_TEMPLATE_LIST
template <typename OnFulfilled>
auto PROMISE::then(OnFulfilled&& on_fulfilled)
    -> std::enable_if_t<
          _::ContinuationTypeTrait<T, OnFulfilled, std::nullptr_t,
                                   std::nullptr_t>::is_valid_continuation,
          Category<typename _::ContinuationTypeTrait<
              T, OnFulfilled, std::nullptr_t, std::nullptr_t>::ResultType>> {
  return then(std::forward<OnFulfilled>(on_fulfilled), default_on_rejected_t{},
              default_on_progress_t{});
}

PROMISE_TEMPLATE_LIST
template <typename OnFulfilled, typename OnRejected>
auto PROMISE::then(OnFulfilled&& on_fulfilled, OnRejected&& on_rejected)
    -> std::enable_if_t<
          _::ContinuationTypeTrait<T, OnFulfilled, OnRejected,
                                   std::nullptr_t>::is_valid_continuation,
          Category<typename _::ContinuationTypeTrait<
              T, OnFulfilled, OnRejected, std::nullptr_t>::ResultType>> {
  return then(std::forward<OnFulfilled>(on_fulfilled),
              std::forward<OnRejected>(on_rejected), default_on_progress_t{});
}

#undef PROMISE_TEMPLATE_LIST
#undef PROMISE

}  // namespace promise