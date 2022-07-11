//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.5
//
// <auto-generated>
//
// Generated from file `Radar.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __Radar_h__
#define __Radar_h__

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

namespace RoboCompRadar
{

class Radar;
class RadarPrx;

}

namespace RoboCompRadar
{

struct RadarData
{
    float distance;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const float&> ice_tuple() const
    {
        return std::tie(distance);
    }
};

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompRadar
{

class Radar : public virtual ::Ice::Object
{
public:

    using ProxyType = RadarPrx;

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

    virtual RadarData getData(const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_getData(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompRadar
{

class RadarPrx : public virtual ::Ice::Proxy<RadarPrx, ::Ice::ObjectPrx>
{
public:

    RadarData getData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompRadar::RadarData>(true, this, &RadarPrx::_iceI_getData, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getDataAsync(const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompRadar::RadarData>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompRadar::RadarData, P>(false, this, &RadarPrx::_iceI_getData, context);
    }

    ::std::function<void()>
    getDataAsync(::std::function<void(::RoboCompRadar::RadarData)> response,
                 ::std::function<void(::std::exception_ptr)> ex = nullptr,
                 ::std::function<void(bool)> sent = nullptr,
                 const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompRadar::RadarData>(std::move(response), std::move(ex), std::move(sent), this, &RoboCompRadar::RadarPrx::_iceI_getData, context);
    }

    /// \cond INTERNAL
    void _iceI_getData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompRadar::RadarData>>&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    RadarPrx() = default;
    friend ::std::shared_ptr<RadarPrx> IceInternal::createProxy<RadarPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits<::RoboCompRadar::RadarData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 4;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamReader<::RoboCompRadar::RadarData, S>
{
    static void read(S* istr, ::RoboCompRadar::RadarData& v)
    {
        istr->readAll(v.distance);
    }
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompRadar
{

using RadarPtr = ::std::shared_ptr<Radar>;
using RadarPrxPtr = ::std::shared_ptr<RadarPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompRadar
{

class Radar;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< Radar>&);
::IceProxy::Ice::Object* upCast(Radar*);
/// \endcond

}

}

namespace RoboCompRadar
{

class Radar;
/// \cond INTERNAL
::Ice::Object* upCast(Radar*);
/// \endcond
typedef ::IceInternal::Handle< Radar> RadarPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompRadar::Radar> RadarPrx;
typedef RadarPrx RadarPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(RadarPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompRadar
{

struct RadarData
{
    ::Ice::Float distance;
};

}

namespace RoboCompRadar
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompRadar::Radar::begin_getData.
 * Create a wrapper instance by calling ::RoboCompRadar::newCallback_Radar_getData.
 */
class Callback_Radar_getData_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Radar_getData_Base> Callback_Radar_getDataPtr;

}

namespace IceProxy
{

namespace RoboCompRadar
{

class Radar : public virtual ::Ice::Proxy<Radar, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompRadar::RadarData getData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getData(_iceI_begin_getData(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getData(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getData(const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getData(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getData(const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getData(context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getData(const ::RoboCompRadar::Callback_Radar_getDataPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getData(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getData(const ::Ice::Context& context, const ::RoboCompRadar::Callback_Radar_getDataPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getData(context, cb, cookie);
    }

    ::RoboCompRadar::RadarData end_getData(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getData(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

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

namespace RoboCompRadar
{

class Radar : public virtual ::Ice::Object
{
public:

    typedef RadarPrx ProxyType;
    typedef RadarPtr PointerType;

    virtual ~Radar();

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

    virtual RadarData getData(const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_getData(::IceInternal::Incoming&, const ::Ice::Current&);
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
inline bool operator==(const Radar& lhs, const Radar& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const Radar& lhs, const Radar& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompRadar::RadarData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 4;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamWriter< ::RoboCompRadar::RadarData, S>
{
    static void write(S* ostr, const ::RoboCompRadar::RadarData& v)
    {
        ostr->write(v.distance);
    }
};

template<typename S>
struct StreamReader< ::RoboCompRadar::RadarData, S>
{
    static void read(S* istr, ::RoboCompRadar::RadarData& v)
    {
        istr->read(v.distance);
    }
};

}
/// \endcond

namespace RoboCompRadar
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompRadar::Radar::begin_getData.
 * Create a wrapper instance by calling ::RoboCompRadar::newCallback_Radar_getData.
 */
template<class T>
class CallbackNC_Radar_getData : public Callback_Radar_getData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const RadarData&);

    CallbackNC_Radar_getData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        RadarPrx proxy = RadarPrx::uncheckedCast(result->getProxy());
        RadarData ret;
        try
        {
            ret = proxy->end_getData(result);
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
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompRadar::Radar::begin_getData.
 */
template<class T> Callback_Radar_getDataPtr
newCallback_Radar_getData(const IceUtil::Handle<T>& instance, void (T::*cb)(const RadarData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Radar_getData<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompRadar::Radar::begin_getData.
 */
template<class T> Callback_Radar_getDataPtr
newCallback_Radar_getData(T* instance, void (T::*cb)(const RadarData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Radar_getData<T>(instance, cb, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompRadar::Radar::begin_getData.
 * Create a wrapper instance by calling ::RoboCompRadar::newCallback_Radar_getData.
 */
template<class T, typename CT>
class Callback_Radar_getData : public Callback_Radar_getData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const RadarData&, const CT&);

    Callback_Radar_getData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        RadarPrx proxy = RadarPrx::uncheckedCast(result->getProxy());
        RadarData ret;
        try
        {
            ret = proxy->end_getData(result);
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
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompRadar::Radar::begin_getData.
 */
template<class T, typename CT> Callback_Radar_getDataPtr
newCallback_Radar_getData(const IceUtil::Handle<T>& instance, void (T::*cb)(const RadarData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Radar_getData<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompRadar::Radar::begin_getData.
 */
template<class T, typename CT> Callback_Radar_getDataPtr
newCallback_Radar_getData(T* instance, void (T::*cb)(const RadarData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Radar_getData<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif
