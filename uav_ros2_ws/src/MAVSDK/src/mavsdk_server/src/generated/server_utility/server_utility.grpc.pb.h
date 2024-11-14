// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: server_utility/server_utility.proto
#ifndef GRPC_server_5futility_2fserver_5futility_2eproto__INCLUDED
#define GRPC_server_5futility_2fserver_5futility_2eproto__INCLUDED

#include "server_utility/server_utility.pb.h"

#include <functional>
#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/client_context.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/proto_utils.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/status.h>
#include <grpcpp/support/stub_options.h>
#include <grpcpp/support/sync_stream.h>

namespace mavsdk {
namespace rpc {
namespace server_utility {

//
// Utility for onboard MAVSDK instances for common "server" tasks.
class ServerUtilityService final {
 public:
  static constexpr char const* service_full_name() {
    return "mavsdk.rpc.server_utility.ServerUtilityService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Sends a statustext.
    virtual ::grpc::Status SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>> AsyncSendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>>(AsyncSendStatusTextRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>> PrepareAsyncSendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>>(PrepareAsyncSendStatusTextRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Sends a statustext.
      virtual void SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>* AsyncSendStatusTextRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::server_utility::SendStatusTextResponse>* PrepareAsyncSendStatusTextRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>> AsyncSendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>>(AsyncSendStatusTextRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>> PrepareAsyncSendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>>(PrepareAsyncSendStatusTextRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response, std::function<void(::grpc::Status)>) override;
      void SendStatusText(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
     private:
      friend class Stub;
      explicit async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class async* async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>* AsyncSendStatusTextRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::server_utility::SendStatusTextResponse>* PrepareAsyncSendStatusTextRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_SendStatusText_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Sends a statustext.
    virtual ::grpc::Status SendStatusText(::grpc::ServerContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_SendStatusText() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSendStatusText(::grpc::ServerContext* context, ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::grpc::ServerAsyncResponseWriter< ::mavsdk::rpc::server_utility::SendStatusTextResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_SendStatusText<Service > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_SendStatusText() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::server_utility::SendStatusTextRequest, ::mavsdk::rpc::server_utility::SendStatusTextResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* request, ::mavsdk::rpc::server_utility::SendStatusTextResponse* response) { return this->SendStatusText(context, request, response); }));}
    void SetMessageAllocatorFor_SendStatusText(
        ::grpc::MessageAllocator< ::mavsdk::rpc::server_utility::SendStatusTextRequest, ::mavsdk::rpc::server_utility::SendStatusTextResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::server_utility::SendStatusTextRequest, ::mavsdk::rpc::server_utility::SendStatusTextResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* SendStatusText(
      ::grpc::CallbackServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_SendStatusText<Service > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_SendStatusText() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_SendStatusText() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSendStatusText(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_SendStatusText() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->SendStatusText(context, request, response); }));
    }
    ~WithRawCallbackMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* SendStatusText(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_SendStatusText : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_SendStatusText() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::mavsdk::rpc::server_utility::SendStatusTextRequest, ::mavsdk::rpc::server_utility::SendStatusTextResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::mavsdk::rpc::server_utility::SendStatusTextRequest, ::mavsdk::rpc::server_utility::SendStatusTextResponse>* streamer) {
                       return this->StreamedSendStatusText(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_SendStatusText() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status SendStatusText(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::server_utility::SendStatusTextRequest* /*request*/, ::mavsdk::rpc::server_utility::SendStatusTextResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedSendStatusText(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mavsdk::rpc::server_utility::SendStatusTextRequest,::mavsdk::rpc::server_utility::SendStatusTextResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_SendStatusText<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_SendStatusText<Service > StreamedService;
};

}  // namespace server_utility
}  // namespace rpc
}  // namespace mavsdk


#endif  // GRPC_server_5futility_2fserver_5futility_2eproto__INCLUDED