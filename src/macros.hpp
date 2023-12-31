#ifndef MACROS_HPP
#define MACROS_HPP

#define BIND_METHOD_0_ARGS(m_class, m_name) \
  ClassDB::bind_method(D_METHOD(#m_name), &m_class::m_name)

#define BIND_METHOD_N_ARGS(m_class, m_name, ...) \
  ClassDB::bind_method(D_METHOD(#m_name, __VA_ARGS__), &m_class::m_name)

#define BIND_METHOD_SELECT(_1, _2, _3, _4, _5, _6, _7, _8, _9, m_macro, ...) m_macro

#define BIND_METHOD(...)  \
  BIND_METHOD_SELECT(     \
      __VA_ARGS__,        \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_N_ARGS, \
      BIND_METHOD_0_ARGS) \
  (__VA_ARGS__)

#define BIND_PROPERTY_HINTED(m_name, m_type, m_hint, m_hint_str) \
  ClassDB::add_property(                                         \
      get_class_static(),                                        \
      PropertyInfo(m_type, m_name, m_hint, m_hint_str),          \
      "set_" m_name,                                             \
      "get_" m_name)

#define BIND_PROPERTY_RANGED(m_name, m_type, m_hint_str) \
  BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_RANGE, m_hint_str)

#define BIND_PROPERTY_ENUM(m_name, m_hint_str) \
  BIND_PROPERTY_HINTED(m_name, Variant::INT, PROPERTY_HINT_ENUM, m_hint_str)

#define BIND_PROPERTY_0(m_name, m_type) BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_NONE, "")

#define BIND_PROPERTY_1(m_name, m_type, m_hint_str) \
  BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_NONE, m_hint_str)

#define BIND_PROPERTY_SELECT(_1, _2, _3, m_macro, ...) m_macro

#define BIND_PROPERTY(...)                                            \
  BIND_PROPERTY_SELECT(__VA_ARGS__, BIND_PROPERTY_1, BIND_PROPERTY_0) \
  (__VA_ARGS__)

#define GPRINTVEC(vector2print) \
  cout << vector2print.x << ", " << vector2print.y << ", " << vector2print.z << "\n";

#define GPRINT(printee) \
  std::cout << printee << "\n";

#endif // MACROS_HPP