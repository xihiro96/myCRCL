//
// Created by mhendrix8 on 7/25/16.
//

#ifndef MY_CRCL_SHARED_PTR_H
#define MY_CRCL_SHARED_PTR_H


#ifdef USE_STD_SHARED_PTR
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#endif

namespace gtri
{

#ifdef USE_STD_SHARED_PTR
template <class T>
using shared_ptr = std::shared_ptr<T>;
#else
template <class T>
using shared_ptr = boost::shared_ptr<T>;
#endif

template<typename T, typename...Args>
shared_ptr<T> make_shared(Args &&...args) {
#ifdef USE_STD_SHARED_PTR
  return std::make_shared<T>(std::forward<Args>(args)...);
#else
  return boost::make_shared<T>(std::forward<Args>(args)...);
#endif
}

}

#endif //MY_CRCL_SHARED_PTR_H
