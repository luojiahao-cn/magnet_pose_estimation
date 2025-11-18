#include <mag_core_utils/param_reader.hpp>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <sstream>

namespace mag_core_utils::param
{
namespace
{
std::string makeContext(const std::string &prefix, const std::string &key)
{
    // 构造形如 a.b.c 的上下文路径，便于错误提示定位
    if (prefix.empty())
    {
        return key;
    }
    return prefix + "." + key;
}

std::string typeName(XmlRpc::XmlRpcValue::Type type)
{
    switch (type)
    {
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
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "datetime";
    case XmlRpc::XmlRpcValue::TypeBase64:
        return "base64";
    case XmlRpc::XmlRpcValue::TypeInvalid:
    default:
        return "invalid";
    }
}

[[noreturn]] void throwMissing(const std::string &context, const std::string &key)
{
    // 当缺少必要字段时抛出带上下文信息的异常
    throw std::runtime_error("parameter '" + makeContext(context, key) + "' is required");
}

[[noreturn]] void throwType(const std::string &context,
                             const std::string &expected,
                             const XmlRpc::XmlRpcValue &value)
{
    // 对类型不匹配的参数给出详细的期望类型说明
    std::ostringstream oss;
    oss << "parameter '" << context << "' must be a " << expected
        << " (got " << typeName(value.getType()) << ")";
    throw std::runtime_error(oss.str());
}

} // namespace

StructReader StructReader::fromParameter(const ros::NodeHandle &nh,
                                         const std::string &key,
                                         const std::string &context_prefix)
{
    namespace rps = rosparam_shortcuts;
    const auto context = makeContext(context_prefix, key);
    std::size_t error = 0;

    // 从 ROS 参数服务器拉取节点，并确保顶层是 struct 类型
    XmlRpc::XmlRpcValue root;
    error += !rps::get(context, nh, key, root);
    rps::shutdownIfError(context, error);

    auto storage = std::make_shared<XmlRpc::XmlRpcValue>(root);

    if (storage->getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        throwType(context, "struct", *storage);
    }

    return StructReader(std::move(storage), storage.get(), context);
}

StructReader::StructReader(std::shared_ptr<XmlRpc::XmlRpcValue> storage,
                           const XmlRpc::XmlRpcValue *value,
                           std::string context)
    : storage_(std::move(storage)), value_(value), context_(std::move(context))
{
    // 构造函数负责保存共享的 XMLRPC 存储以及当前节点指针
    if (!value_)
    {
        throw std::runtime_error("null XmlRpc struct at context '" + context_ + "'");
    }
    if (value_->getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        throwType(context_, "struct", *value_);
    }
}

const XmlRpc::XmlRpcValue &StructReader::getMember(const std::string &key) const
{
    // 安全访问子成员，缺失时给出上下文
    if (!value_->hasMember(key))
    {
        throwMissing(context_, key);
    }
    return (*value_)[key];
}

StructReader StructReader::childStruct(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        throwType(context, "struct", child);
    }
    // 递归包装子结构体，保留共享的 XMLRPC 存储
    return StructReader(storage_, &child, context);
}

ArrayReader StructReader::childArray(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        throwType(context, "array", child);
    }
    return ArrayReader(storage_, &child, context);
}

bool StructReader::has(const std::string &key) const
{
    // 对 optional 字段执行存在性测试
    return value_->hasMember(key);
}

std::string StructReader::requireString(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        throwType(context, "string", child);
    }
    return static_cast<std::string>(child);
}

std::string StructReader::optionalString(const std::string &key, const std::string &default_value) const
{
    if (!has(key))
    {
        return default_value;
    }
    return requireString(key);
}

double StructReader::requireNumber(const std::string &key) const
{
    // 统一处理 int/double 两种数值类型，保证调用方得到 double
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(child);
    }
    if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(child);
    }
    throwType(context, "number", child);
    return 0.0; // unreachable
}

double StructReader::optionalNumber(const std::string &key, double default_value) const
{
    if (!has(key))
    {
        return default_value;
    }
    return requireNumber(key);
}

int StructReader::requireInt(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(child);
    }
    if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<int>(static_cast<double>(child));
    }
    throwType(context, "int", child);
    return 0; // unreachable
}

bool StructReader::requireBool(const std::string &key) const
{
    // 将 0/1 数值和显式布尔都映射为 bool
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
        return static_cast<bool>(child);
    }
    if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(child) != 0;
    }
    if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(child) != 0.0;
    }
    throwType(context, "bool", child);
    return false; // unreachable
}

bool StructReader::optionalBool(const std::string &key, bool default_value) const
{
    if (!has(key))
    {
        return default_value;
    }
    return requireBool(key);
}

std::vector<double> StructReader::requireVector3(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeArray || child.size() != 3)
    {
        throw std::runtime_error("parameter '" + context + "' must be an array of length 3");
    }
    // 依次读取数组元素，并兼容 int/double 两种数值类型
    std::vector<double> result(3);
    for (int i = 0; i < child.size(); ++i)
    {
        const auto element_context = context + "[" + std::to_string(i) + "]";
        const auto &element = child[i];
        if (element.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            result[i] = static_cast<double>(element);
        }
        else if (element.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            result[i] = static_cast<int>(element);
        }
        else
        {
            throwType(element_context, "number", element);
        }
    }
    return result;
}

ArrayReader::ArrayReader(std::shared_ptr<XmlRpc::XmlRpcValue> storage,
                         const XmlRpc::XmlRpcValue *value,
                         std::string context)
    : storage_(std::move(storage)), value_(value), context_(std::move(context))
{
    // 记录数组节点并在构造阶段验证类型
    if (!value_)
    {
        throw std::runtime_error("null XmlRpc array at context '" + context_ + "'");
    }
    if (value_->getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        throwType(context_, "array", *value_);
    }
}

int ArrayReader::size() const
{
    return value_->size();
}

void ArrayReader::requireIndexInRange(int index) const
{
    // 数组越界保护，同时给出当前上下文信息
    if (index < 0 || index >= size())
    {
        std::ostringstream oss;
        oss << "index " << index << " out of range for parameter '" << context_
            << "' (size " << size() << ")";
        throw std::runtime_error(oss.str());
    }
}

std::string ArrayReader::elementContext(int index) const
{
    return context_ + "[" + std::to_string(index) + "]";
}

const XmlRpc::XmlRpcValue &ArrayReader::rawAt(int index) const
{
    requireIndexInRange(index);
    return (*value_)[index];
}

StructReader ArrayReader::structAt(int index) const
{
    const auto &child = rawAt(index);
    const auto context = elementContext(index);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        throwType(context, "struct", child);
    }
    // 重用 StructReader 帮助类读取数组元素中的对象
    return StructReader(storage_, &child, context);
}

double ArrayReader::numberAt(int index) const
{
    const auto &child = rawAt(index);
    const auto context = elementContext(index);
    if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(child);
    }
    if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(child);
    }
    throwType(context, "number", child);
    return 0.0; // unreachable
}

std::vector<double> ArrayReader::vector3At(int index) const
{
    const auto &child = rawAt(index);
    const auto context = elementContext(index);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeArray || child.size() != 3)
    {
        std::ostringstream oss;
        oss << "parameter '" << context << "' must be an array of length 3";
        throw std::runtime_error(oss.str());
    }
    // 将数组元素转换为 double，匹配 requireVector3 的逻辑
    std::vector<double> result(3);
    for (int i = 0; i < child.size(); ++i)
    {
        const auto &element = child[i];
        const auto element_context = context + "[" + std::to_string(i) + "]";
        if (element.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            result[i] = static_cast<double>(element);
        }
        else if (element.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            result[i] = static_cast<int>(element);
        }
        else
        {
            throwType(element_context, "number", element);
        }
    }
    return result;
}

} // namespace mag_core_utils::param
