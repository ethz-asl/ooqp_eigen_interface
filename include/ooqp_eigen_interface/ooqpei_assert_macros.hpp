/**
 * @file   ooqpei_assert_macros.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Mon Dec 12 11:22:20 2011
 *
 * @brief  Assert macros to facilitate rapid prototyping. Use them early and often.
 *
 *
 */

#ifndef OOQPEI_ASSERT_MACROS_HPP_
#define OOQPEI_ASSERT_MACROS_HPP_


#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "ooqpei_source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define OOQPEI_DEFINE_EXCEPTION(exceptionName, exceptionParent)       \
  class exceptionName : public exceptionParent {            \
  public:                               \
  exceptionName(const char * message) : exceptionParent(message) {}   \
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}                 \
  };


namespace ooqpei {
namespace common {
  namespace internal {

    template<typename OOQPEI_EXCEPTION_T>
    inline void ooqpei_throw_exception(std::string const & exceptionType, ooqpei::source_file_pos sfp, std::string const & message)
    {
      std::stringstream ooqpei_assert_stringstream;
#ifdef _WIN32
      // I have no idea what broke this on Windows but it doesn't work with the << operator.
      ooqpei_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
      ooqpei_assert_stringstream << exceptionType <<  sfp << " " << message;
#endif
      throw(OOQPEI_EXCEPTION_T(ooqpei_assert_stringstream.str()));
    }

    template<typename OOQPEI_EXCEPTION_T>
    inline void ooqpei_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
                   int line, std::string const & message)
    {
      ooqpei_throw_exception<OOQPEI_EXCEPTION_T>(exceptionType, ooqpei::source_file_pos(function,file,line),message);
    }


  } // namespace internal

  template<typename OOQPEI_EXCEPTION_T>
  inline void ooqpei_assert_throw(bool assert_condition, std::string message, ooqpei::source_file_pos sfp) {
    if(!assert_condition)
      {
    internal::ooqpei_throw_exception<OOQPEI_EXCEPTION_T>("", sfp,message);
      }
  }


} // namespace common
} // namespace rm




#define OOQPEI_THROW(exceptionType, message) {                \
    std::stringstream ooqpei_assert_stringstream;             \
    ooqpei_assert_stringstream << message;                  \
    ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
  }


#define OOQPEI_THROW_SFP(exceptionType, SourceFilePos, message){      \
    std::stringstream ooqpei_assert_stringstream;             \
    ooqpei_assert_stringstream << message;                  \
    ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, ooqpei_assert_stringstream.str()); \
  }

#define OOQPEI_ASSERT_TRUE(exceptionType, condition, message)       \
  if(!(condition))                            \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_FALSE(exceptionType, condition, message)        \
  if((condition))                           \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_LT(exceptionType, value, upperBound, message)     \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_GE(exceptionType, value, lowerBound, message)     \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_LE(exceptionType, value, upperBound, message)     \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_GT(exceptionType, value, lowerBound, message)     \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_EQ(exceptionType, value, testValue, message)      \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_NE(exceptionType, value, testValue, message)      \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define OOQPEI_THROW_DBG(exceptionType, message){             \
    std::stringstream ooqpei_assert_stringstream;             \
    ooqpei_assert_stringstream << message;                  \
    ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
  }



#define OOQPEI_ASSERT_TRUE_DBG(exceptionType, condition, message)     \
  if(!(condition))                            \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_FALSE_DBG(exceptionType, condition, message)      \
  if((condition))                           \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, ooqpei_assert_stringstream.str()); \
    }


#define OOQPEI_ASSERT_DBG_RE( condition, message) OOQPEI_ASSERT_DBG(std::runtime_error, condition, message)

#define OOQPEI_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_LT_DBG(exceptionType, value, upperBound, message)   \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)   \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_LE_DBG(exceptionType, value, upperBound, message)   \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }

#define OOQPEI_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)   \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_EQ_DBG(exceptionType, value, testValue, message)    \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }


#define OOQPEI_ASSERT_NE_DBG(exceptionType, value, testValue, message)    \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }



#define OOQPEI_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream ooqpei_assert_stringstream;             \
      ooqpei_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      ooqpei::common::internal::ooqpei_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,ooqpei_assert_stringstream.str()); \
    }


#define OOQPEI_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define OOQPEI_OUT(X)
#define OOQPEI_THROW_DBG(exceptionType, message)
#define OOQPEI_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define OOQPEI_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define OOQPEI_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define OOQPEI_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define OOQPEI_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define OOQPEI_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define OOQPEI_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define OOQPEI_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define OOQPEI_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define OOQPEI_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)
#endif

#endif /* OOQPEI_ASSERT_MACROS_HPP_ */
