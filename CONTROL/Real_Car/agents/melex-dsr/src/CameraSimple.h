//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.5
//
// <auto-generated>
//
// Generated from file `CameraSimple.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __CameraSimple_h__
#define __CameraSimple_h__

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

namespace RoboCompCameraSimple
{

class CameraSimple;
class CameraSimplePrx;

}

namespace RoboCompCameraSimple
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

using ImgType = ::std::vector<::Ice::Byte>;

struct TImage
{
    bool compressed;
    int width;
    int height;
    int depth;
    ::RoboCompCameraSimple::ImgType image;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const bool&, const int&, const int&, const int&, const ::RoboCompCameraSimple::ImgType&> ice_tuple() const
    {
        return std::tie(compressed, width, height, depth, image);
    }
};

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompCameraSimple
{

class CameraSimple : public virtual ::Ice::Object
{
public:

    using ProxyType = CameraSimplePrx;

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

    virtual TImage getImage(const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_getImage(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompCameraSimple
{

class CameraSimplePrx : public virtual ::Ice::Proxy<CameraSimplePrx, ::Ice::ObjectPrx>
{
public:

    TImage getImage(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompCameraSimple::TImage>(true, this, &CameraSimplePrx::_iceI_getImage, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getImageAsync(const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompCameraSimple::TImage>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompCameraSimple::TImage, P>(false, this, &CameraSimplePrx::_iceI_getImage, context);
    }

    ::std::function<void()>
    getImageAsync(::std::function<void(::RoboCompCameraSimple::TImage)> response,
                  ::std::function<void(::std::exception_ptr)> ex = nullptr,
                  ::std::function<void(bool)> sent = nullptr,
                  const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompCameraSimple::TImage>(std::move(response), std::move(ex), std::move(sent), this, &RoboCompCameraSimple::CameraSimplePrx::_iceI_getImage, context);
    }

    /// \cond INTERNAL
    void _iceI_getImage(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompCameraSimple::TImage>>&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    CameraSimplePrx() = default;
    friend ::std::shared_ptr<CameraSimplePrx> IceInternal::createProxy<CameraSimplePrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<typename S>
struct StreamReader<::RoboCompCameraSimple::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompCameraSimple::HardwareFailedException& v)
    {
        istr->readAll(v.what);
    }
};

template<>
struct StreamableTraits<::RoboCompCameraSimple::TImage>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 14;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompCameraSimple::TImage, S>
{
    static void read(S* istr, ::RoboCompCameraSimple::TImage& v)
    {
        istr->readAll(v.compressed, v.width, v.height, v.depth, v.image);
    }
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompCameraSimple
{

using CameraSimplePtr = ::std::shared_ptr<CameraSimple>;
using CameraSimplePrxPtr = ::std::shared_ptr<CameraSimplePrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompCameraSimple
{

class CameraSimple;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< CameraSimple>&);
::IceProxy::Ice::Object* upCast(CameraSimple*);
/// \endcond

}

}

namespace RoboCompCameraSimple
{

class CameraSimple;
/// \cond INTERNAL
::Ice::Object* upCast(CameraSimple*);
/// \endcond
typedef ::IceInternal::Handle< CameraSimple> CameraSimplePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompCameraSimple::CameraSimple> CameraSimplePrx;
typedef CameraSimplePrx CameraSimplePrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(CameraSimplePtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompCameraSimple
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

typedef ::std::vector< ::Ice::Byte> ImgType;

struct TImage
{
    bool compressed;
    ::Ice::Int width;
    ::Ice::Int height;
    ::Ice::Int depth;
    ::RoboCompCameraSimple::ImgType image;
};

}

namespace RoboCompCameraSimple
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 * Create a wrapper instance by calling ::RoboCompCameraSimple::newCallback_CameraSimple_getImage.
 */
class Callback_CameraSimple_getImage_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_CameraSimple_getImage_Base> Callback_CameraSimple_getImagePtr;

}

namespace IceProxy
{

namespace RoboCompCameraSimple
{

class CameraSimple : public virtual ::Ice::Proxy<CameraSimple, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompCameraSimple::TImage getImage(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getImage(_iceI_begin_getImage(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getImage(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getImage(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getImage(const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getImage(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getImage(const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getImage(context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getImage(const ::RoboCompCameraSimple::Callback_CameraSimple_getImagePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getImage(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getImage(const ::Ice::Context& context, const ::RoboCompCameraSimple::Callback_CameraSimple_getImagePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getImage(context, cb, cookie);
    }

    ::RoboCompCameraSimple::TImage end_getImage(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getImage(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

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

namespace RoboCompCameraSimple
{

class CameraSimple : public virtual ::Ice::Object
{
public:

    typedef CameraSimplePrx ProxyType;
    typedef CameraSimplePtr PointerType;

    virtual ~CameraSimple();

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

    virtual TImage getImage(const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_getImage(::IceInternal::Incoming&, const ::Ice::Current&);
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
inline bool operator==(const CameraSimple& lhs, const CameraSimple& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const CameraSimple& lhs, const CameraSimple& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompCameraSimple::HardwareFailedException>
{
    static const StreamHelperCategory helper = StreamHelperCategoryUserException;
};

template<typename S>
struct StreamWriter< ::RoboCompCameraSimple::HardwareFailedException, S>
{
    static void write(S* ostr, const ::RoboCompCameraSimple::HardwareFailedException& v)
    {
        ostr->write(v.what);
    }
};

template<typename S>
struct StreamReader< ::RoboCompCameraSimple::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompCameraSimple::HardwareFailedException& v)
    {
        istr->read(v.what);
    }
};

template<>
struct StreamableTraits< ::RoboCompCameraSimple::TImage>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 14;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompCameraSimple::TImage, S>
{
    static void write(S* ostr, const ::RoboCompCameraSimple::TImage& v)
    {
        ostr->write(v.compressed);
        ostr->write(v.width);
        ostr->write(v.height);
        ostr->write(v.depth);
        ostr->write(v.image);
    }
};

template<typename S>
struct StreamReader< ::RoboCompCameraSimple::TImage, S>
{
    static void read(S* istr, ::RoboCompCameraSimple::TImage& v)
    {
        istr->read(v.compressed);
        istr->read(v.width);
        istr->read(v.height);
        istr->read(v.depth);
        istr->read(v.image);
    }
};

}
/// \endcond

namespace RoboCompCameraSimple
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 * Create a wrapper instance by calling ::RoboCompCameraSimple::newCallback_CameraSimple_getImage.
 */
template<class T>
class CallbackNC_CameraSimple_getImage : public Callback_CameraSimple_getImage_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const TImage&);

    CallbackNC_CameraSimple_getImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        CameraSimplePrx proxy = CameraSimplePrx::uncheckedCast(result->getProxy());
        TImage ret;
        try
        {
            ret = proxy->end_getImage(result);
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
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 */
template<class T> Callback_CameraSimple_getImagePtr
newCallback_CameraSimple_getImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const TImage&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CameraSimple_getImage<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 */
template<class T> Callback_CameraSimple_getImagePtr
newCallback_CameraSimple_getImage(T* instance, void (T::*cb)(const TImage&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CameraSimple_getImage<T>(instance, cb, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 * Create a wrapper instance by calling ::RoboCompCameraSimple::newCallback_CameraSimple_getImage.
 */
template<class T, typename CT>
class Callback_CameraSimple_getImage : public Callback_CameraSimple_getImage_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const TImage&, const CT&);

    Callback_CameraSimple_getImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        CameraSimplePrx proxy = CameraSimplePrx::uncheckedCast(result->getProxy());
        TImage ret;
        try
        {
            ret = proxy->end_getImage(result);
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
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 */
template<class T, typename CT> Callback_CameraSimple_getImagePtr
newCallback_CameraSimple_getImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const TImage&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CameraSimple_getImage<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCameraSimple::CameraSimple::begin_getImage.
 */
template<class T, typename CT> Callback_CameraSimple_getImagePtr
newCallback_CameraSimple_getImage(T* instance, void (T::*cb)(const TImage&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CameraSimple_getImage<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif