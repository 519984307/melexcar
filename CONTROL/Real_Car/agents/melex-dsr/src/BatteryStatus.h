//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.5
//
// <auto-generated>
//
// Generated from file `BatteryStatus.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __BatteryStatus_h__
#define __BatteryStatus_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 5
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompBatteryStatus
{

class BatteryStatus;
class BatteryStatusPrx;

}

namespace RoboCompBatteryStatus
{

class HardwareFailedException : public ::Ice::UserExceptionHelper<HardwareFailedException, ::Ice::UserException>
{
public:

    virtual ~HardwareFailedException();

    HardwareFailedException(const HardwareFailedException&) = default;

    HardwareFailedException() = default;

    /**
     * One-shot constructor to initialize all data members.
     */
    HardwareFailedException(const ::std::string& what) :
        what(what)
    {
    }

    /**
     * Obtains a tuple containing all of the exception's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&> ice_tuple() const
    {
        return std::tie(what);
    }

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    ::std::string what;
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

struct TBattery
{
    float percentage;
    float voltage;
    float current;
    float timetogo;
    float power;
    float consumptionperhour;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const float&, const float&, const float&, const float&, const float&, const float&> ice_tuple() const
    {
        return std::tie(percentage, voltage, current, timetogo, power, consumptionperhour);
    }
};

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompBatteryStatus
{

class BatteryStatus : public virtual ::Ice::Object
{
public:

    using ProxyType = BatteryStatusPrx;

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(::std::string id, const ::Ice::Current& current) const override;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current& current) const override;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual ::std::string ice_id(const ::Ice::Current& current) const override;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual TBattery getBatteryState(const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_getBatteryState(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompBatteryStatus
{

class BatteryStatusPrx : public virtual ::Ice::Proxy<BatteryStatusPrx, ::Ice::ObjectPrx>
{
public:

    TBattery getBatteryState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompBatteryStatus::TBattery>(true, this, &BatteryStatusPrx::_iceI_getBatteryState, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getBatteryStateAsync(const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompBatteryStatus::TBattery>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompBatteryStatus::TBattery, P>(false, this, &BatteryStatusPrx::_iceI_getBatteryState, context);
    }

    ::std::function<void()>
    getBatteryStateAsync(::std::function<void(::RoboCompBatteryStatus::TBattery)> response,
                         ::std::function<void(::std::exception_ptr)> ex = nullptr,
                         ::std::function<void(bool)> sent = nullptr,
                         const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompBatteryStatus::TBattery>(std::move(response), std::move(ex), std::move(sent), this, &RoboCompBatteryStatus::BatteryStatusPrx::_iceI_getBatteryState, context);
    }

    /// \cond INTERNAL
    void _iceI_getBatteryState(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompBatteryStatus::TBattery>>&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    BatteryStatusPrx() = default;
    friend ::std::shared_ptr<BatteryStatusPrx> IceInternal::createProxy<BatteryStatusPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<typename S>
struct StreamReader<::RoboCompBatteryStatus::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompBatteryStatus::HardwareFailedException& v)
    {
        istr->readAll(v.what);
    }
};

template<>
struct StreamableTraits<::RoboCompBatteryStatus::TBattery>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 24;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamReader<::RoboCompBatteryStatus::TBattery, S>
{
    static void read(S* istr, ::RoboCompBatteryStatus::TBattery& v)
    {
        istr->readAll(v.percentage, v.voltage, v.current, v.timetogo, v.power, v.consumptionperhour);
    }
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompBatteryStatus
{

using BatteryStatusPtr = ::std::shared_ptr<BatteryStatus>;
using BatteryStatusPrxPtr = ::std::shared_ptr<BatteryStatusPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompBatteryStatus
{

class BatteryStatus;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< BatteryStatus>&);
::IceProxy::Ice::Object* upCast(BatteryStatus*);
/// \endcond

}

}

namespace RoboCompBatteryStatus
{

class BatteryStatus;
/// \cond INTERNAL
::Ice::Object* upCast(BatteryStatus*);
/// \endcond
typedef ::IceInternal::Handle< BatteryStatus> BatteryStatusPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompBatteryStatus::BatteryStatus> BatteryStatusPrx;
typedef BatteryStatusPrx BatteryStatusPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(BatteryStatusPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompBatteryStatus
{

class HardwareFailedException : public ::Ice::UserException
{
public:

    HardwareFailedException() {}
    /**
     * One-shot constructor to initialize all data members.
     */
    explicit HardwareFailedException(const ::std::string& what);
    virtual ~HardwareFailedException() throw();

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    virtual ::std::string ice_id() const;
    /**
     * Polymporphically clones this exception.
     * @return A shallow copy of this exception.
     */
    virtual HardwareFailedException* ice_clone() const;
    /**
     * Throws this exception.
     */
    virtual void ice_throw() const;

    ::std::string what;

protected:

    /// \cond STREAM
    virtual void _writeImpl(::Ice::OutputStream*) const;
    virtual void _readImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

struct TBattery
{
    ::Ice::Float percentage;
    ::Ice::Float voltage;
    ::Ice::Float current;
    ::Ice::Float timetogo;
    ::Ice::Float power;
    ::Ice::Float consumptionperhour;
};

}

namespace RoboCompBatteryStatus
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 * Create a wrapper instance by calling ::RoboCompBatteryStatus::newCallback_BatteryStatus_getBatteryState.
 */
class Callback_BatteryStatus_getBatteryState_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_BatteryStatus_getBatteryState_Base> Callback_BatteryStatus_getBatteryStatePtr;

}

namespace IceProxy
{

namespace RoboCompBatteryStatus
{

class BatteryStatus : public virtual ::Ice::Proxy<BatteryStatus, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompBatteryStatus::TBattery getBatteryState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getBatteryState(_iceI_begin_getBatteryState(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getBatteryState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getBatteryState(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getBatteryState(const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getBatteryState(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getBatteryState(const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getBatteryState(context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getBatteryState(const ::RoboCompBatteryStatus::Callback_BatteryStatus_getBatteryStatePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getBatteryState(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getBatteryState(const ::Ice::Context& context, const ::RoboCompBatteryStatus::Callback_BatteryStatus_getBatteryStatePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getBatteryState(context, cb, cookie);
    }

    ::RoboCompBatteryStatus::TBattery end_getBatteryState(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getBatteryState(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    /**
     * Obtains the Slice type ID corresponding to this interface.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:
    /// \cond INTERNAL

    virtual ::IceProxy::Ice::Object* _newInstance() const;
    /// \endcond
};

}

}

namespace RoboCompBatteryStatus
{

class BatteryStatus : public virtual ::Ice::Object
{
public:

    typedef BatteryStatusPrx ProxyType;
    typedef BatteryStatusPtr PointerType;

    virtual ~BatteryStatus();

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(const ::std::string& id, const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual const ::std::string& ice_id(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual TBattery getBatteryState(const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_getBatteryState(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

protected:

    /// \cond STREAM
    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
inline bool operator==(const BatteryStatus& lhs, const BatteryStatus& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const BatteryStatus& lhs, const BatteryStatus& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompBatteryStatus::HardwareFailedException>
{
    static const StreamHelperCategory helper = StreamHelperCategoryUserException;
};

template<typename S>
struct StreamWriter< ::RoboCompBatteryStatus::HardwareFailedException, S>
{
    static void write(S* ostr, const ::RoboCompBatteryStatus::HardwareFailedException& v)
    {
        ostr->write(v.what);
    }
};

template<typename S>
struct StreamReader< ::RoboCompBatteryStatus::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompBatteryStatus::HardwareFailedException& v)
    {
        istr->read(v.what);
    }
};

template<>
struct StreamableTraits< ::RoboCompBatteryStatus::TBattery>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 24;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamWriter< ::RoboCompBatteryStatus::TBattery, S>
{
    static void write(S* ostr, const ::RoboCompBatteryStatus::TBattery& v)
    {
        ostr->write(v.percentage);
        ostr->write(v.voltage);
        ostr->write(v.current);
        ostr->write(v.timetogo);
        ostr->write(v.power);
        ostr->write(v.consumptionperhour);
    }
};

template<typename S>
struct StreamReader< ::RoboCompBatteryStatus::TBattery, S>
{
    static void read(S* istr, ::RoboCompBatteryStatus::TBattery& v)
    {
        istr->read(v.percentage);
        istr->read(v.voltage);
        istr->read(v.current);
        istr->read(v.timetogo);
        istr->read(v.power);
        istr->read(v.consumptionperhour);
    }
};

}
/// \endcond

namespace RoboCompBatteryStatus
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 * Create a wrapper instance by calling ::RoboCompBatteryStatus::newCallback_BatteryStatus_getBatteryState.
 */
template<class T>
class CallbackNC_BatteryStatus_getBatteryState : public Callback_BatteryStatus_getBatteryState_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const TBattery&);

    CallbackNC_BatteryStatus_getBatteryState(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        BatteryStatusPrx proxy = BatteryStatusPrx::uncheckedCast(result->getProxy());
        TBattery ret;
        try
        {
            ret = proxy->end_getBatteryState(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret);
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 */
template<class T> Callback_BatteryStatus_getBatteryStatePtr
newCallback_BatteryStatus_getBatteryState(const IceUtil::Handle<T>& instance, void (T::*cb)(const TBattery&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_BatteryStatus_getBatteryState<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 */
template<class T> Callback_BatteryStatus_getBatteryStatePtr
newCallback_BatteryStatus_getBatteryState(T* instance, void (T::*cb)(const TBattery&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_BatteryStatus_getBatteryState<T>(instance, cb, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 * Create a wrapper instance by calling ::RoboCompBatteryStatus::newCallback_BatteryStatus_getBatteryState.
 */
template<class T, typename CT>
class Callback_BatteryStatus_getBatteryState : public Callback_BatteryStatus_getBatteryState_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const TBattery&, const CT&);

    Callback_BatteryStatus_getBatteryState(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        BatteryStatusPrx proxy = BatteryStatusPrx::uncheckedCast(result->getProxy());
        TBattery ret;
        try
        {
            ret = proxy->end_getBatteryState(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, CT::dynamicCast(result->getCookie()));
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 */
template<class T, typename CT> Callback_BatteryStatus_getBatteryStatePtr
newCallback_BatteryStatus_getBatteryState(const IceUtil::Handle<T>& instance, void (T::*cb)(const TBattery&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_BatteryStatus_getBatteryState<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompBatteryStatus::BatteryStatus::begin_getBatteryState.
 */
template<class T, typename CT> Callback_BatteryStatus_getBatteryStatePtr
newCallback_BatteryStatus_getBatteryState(T* instance, void (T::*cb)(const TBattery&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_BatteryStatus_getBatteryState<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif