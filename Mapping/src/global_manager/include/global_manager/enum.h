#ifndef MAP_MERGE_ENUM_H_
#define MAP_MERGE_ENUM_H_

/**
 * Support for enum  with string conversions.
 */

/// @cond DOXYGEN_SKIP

#include <ostream>
#include <type_traits>

#define NUM_ARGS_(_10, _9, _8, _7, _6, _5, _4, _3, _2, _1, N, ...) N
#define NUM_ARGS(...) NUM_ARGS_(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define FOREACH_MACRO(MACRO, ...)                                              \
  FOREACH_(NUM_ARGS(__VA_ARGS__), MACRO, __VA_ARGS__)
#define FOREACH_(N, M, ...) FOREACH_x(N, M, __VA_ARGS__)
#define FOREACH_x(N, M, ...) FOREACH_##N(M, __VA_ARGS__)
#define FOREACH_1(M, A) M(A)
#define FOREACH_2(M, A, ...) M(A), FOREACH_1(M, __VA_ARGS__)
#define FOREACH_3(M, A, ...) M(A), FOREACH_2(M, __VA_ARGS__)
#define FOREACH_4(M, A, ...) M(A), FOREACH_3(M, __VA_ARGS__)
#define FOREACH_5(M, A, ...) M(A), FOREACH_4(M, __VA_ARGS__)
#define FOREACH_6(M, A, ...) M(A), FOREACH_5(M, __VA_ARGS__)
#define FOREACH_7(M, A, ...) M(A), FOREACH_6(M, __VA_ARGS__)
#define FOREACH_8(M, A, ...) M(A), FOREACH_7(M, __VA_ARGS__)
#define STRINGIFY_(X) #X
#define STRINGIFY(X) STRINGIFY_(X)

#define ENUM_CLASS(EnumType, ...)                                              \
  enum class EnumType { __VA_ARGS__ };                                         \
                                                                               \
  namespace enums                                                              \
  {                                                                            \
  constexpr inline static const char *to_string(EnumType e)                    \
  {                                                                            \
    const char *names[]{FOREACH_MACRO(STRINGIFY, __VA_ARGS__)};                \
    static_assert(NUM_ARGS(__VA_ARGS__) == (sizeof(names) / sizeof(names[0])), \
                  "unsupported number of enum literals");                      \
    return names[static_cast<std::underlying_type_t<EnumType>>(e)];            \
  }                                                                            \
                                                                               \
  template <typename T>                                                        \
  static std::enable_if_t<std::is_same<T, EnumType>::value, T>                 \
  from_string(const std::string &s)                                            \
  {                                                                            \
    constexpr const static char *names[]{                                      \
        FOREACH_MACRO(STRINGIFY, __VA_ARGS__)};                                \
    static_assert(NUM_ARGS(__VA_ARGS__) == (sizeof(names) / sizeof(names[0])), \
                  "unsupported number of enum literals");                      \
    std::underlying_type_t<EnumType> i = 0;                                    \
    for (auto name : names) {                                                  \
      if (name == s) {                                                         \
        return static_cast<EnumType>(i);                                       \
      }                                                                        \
      ++i;                                                                     \
    }                                                                          \
    throw std::runtime_error("from_string: " + s +                             \
                             " is invalid value for "                          \
                             "enum " #EnumType);                               \
  }                                                                            \
  } /* namespace enums */                                                      \
  static inline std::ostream &operator<<(std::ostream &stream, EnumType value) \
  {                                                                            \
    stream << enums::to_string(value);                                         \
    return stream;                                                             \
  }

/// @endcond DOXYGEN_SKIP

#endif  // MAP_MERGE_ENUM_H_
