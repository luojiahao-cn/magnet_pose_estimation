#include "mag_core_utils/xmlrpc_utils.hpp"

#include <sstream>
#include <stdexcept>

namespace mag_core_utils::xmlrpc {
namespace {

std::string typeName(XmlRpc::XmlRpcValue::Type type) {
  switch (type) {
    case XmlRpc::XmlRpcValue::TypeStruct:
      return "struct";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "array";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "int";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "bool";
    default:
      return "invalid";
  }
}

[[noreturn]] void throwMissing(const std::string &context) {
  throw std::runtime_error("缺少参数: " + context);
}

[[noreturn]] void throwType(const std::string &context,
                             const std::string &expected,
                             XmlRpc::XmlRpcValue::Type type) {
  std::ostringstream oss;
  oss << context << " 必须是 " << expected << " (当前类型=" << typeName(type) << ")";
  throw std::runtime_error(oss.str());
}

}  // namespace

std::string makeContext(const std::string &context, const std::string &member) {
  if (context.empty()) {
    return member;
  }
  if (member.empty()) {
    return context;
  }
  if (!member.empty() && member.front() == '[') {
    return context + member;
  }
  return context + "." + member;
}

const XmlRpc::XmlRpcValue &requireMember(const XmlRpc::XmlRpcValue &node,
                                         const std::string &member,
                                         const std::string &context) {
  if (!node.hasMember(member)) {
    throwMissing(makeContext(context, member));
  }
  return node[member];
}

bool hasMember(const XmlRpc::XmlRpcValue &node, const std::string &member) {
  return node.hasMember(member);
}

const XmlRpc::XmlRpcValue &requireStructField(const XmlRpc::XmlRpcValue &node,
                                              const std::string &member,
                                              const std::string &context) {
  const auto &child = requireMember(node, member, context);
  if (child.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throwType(makeContext(context, member), "struct", child.getType());
  }
  return child;
}

const XmlRpc::XmlRpcValue &requireArrayField(const XmlRpc::XmlRpcValue &node,
                                             const std::string &member,
                                             const std::string &context) {
  const auto &child = requireMember(node, member, context);
  if (child.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throwType(makeContext(context, member), "array", child.getType());
  }
  return child;
}

std::string requireStringField(const XmlRpc::XmlRpcValue &node,
                               const std::string &member,
                               const std::string &context) {
  const auto &child = requireMember(node, member, context);
  if (child.getType() != XmlRpc::XmlRpcValue::TypeString) {
    throwType(makeContext(context, member), "string", child.getType());
  }
  return static_cast<std::string>(child);
}

std::string optionalStringField(const XmlRpc::XmlRpcValue &node,
                                const std::string &member,
                                const std::string &context,
                                const std::string &default_value) {
  if (!hasMember(node, member)) {
    return default_value;
  }
  return requireStringField(node, member, context);
}

double readNumber(const XmlRpc::XmlRpcValue &value,
                  const std::string &context) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throwType(context, "number", value.getType());
  return 0.0;
}

double optionalNumberField(const XmlRpc::XmlRpcValue &node,
                           const std::string &member,
                           const std::string &context,
                           double default_value) {
  if (!hasMember(node, member)) {
    return default_value;
  }
  const auto &child = requireMember(node, member, context);
  return readNumber(child, makeContext(context, member));
}

bool optionalBoolField(const XmlRpc::XmlRpcValue &node,
                       const std::string &member,
                       const std::string &context,
                       bool default_value) {
  if (!hasMember(node, member)) {
    return default_value;
  }
  const auto &child = requireMember(node, member, context);
  if (child.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(child);
  }
  if (child.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(child) != 0;
  }
  if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(child) != 0.0;
  }
  throwType(makeContext(context, member), "bool", child.getType());
  return default_value;
}

std::vector<double> readVector3(const XmlRpc::XmlRpcValue &value,
                                const std::string &context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) {
    throw std::runtime_error(context + " 必须是长度为 3 的数组");
  }
  std::vector<double> result(3);
  for (int i = 0; i < 3; ++i) {
    result[i] = readNumber(value[i], makeContext(context, "[" + std::to_string(i) + "]"));
  }
  return result;
}

std::vector<double> requireVector3Field(const XmlRpc::XmlRpcValue &node,
                                        const std::string &member,
                                        const std::string &context) {
  const auto &child = requireMember(node, member, context);
  return readVector3(child, makeContext(context, member));
}

const XmlRpc::XmlRpcValue &asStruct(const XmlRpc::XmlRpcValue &value,
                                    const std::string &context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throwType(context, "struct", value.getType());
  }
  return value;
}

}  // namespace mag_core_utils::xmlrpc
