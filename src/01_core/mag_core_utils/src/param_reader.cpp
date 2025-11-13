#include <mag_core_utils/param_reader.hpp>

#include <sstream>

namespace mag_core_utils::param
{
namespace
{
std::string makeContext(const std::string &prefix, const std::string &key)
{
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
    throw std::runtime_error("parameter '" + makeContext(context, key) + "' is required");
}

[[noreturn]] void throwType(const std::string &context,
                             const std::string &expected,
                             const XmlRpc::XmlRpcValue &value)
{
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
    XmlRpc::XmlRpcValue root;
    if (!nh.getParam(key, root))
    {
        throw std::runtime_error("missing parameter '" + makeContext(context_prefix, key) + "'");
    }

    auto storage = std::make_shared<XmlRpc::XmlRpcValue>(root);
    const auto context = makeContext(context_prefix, key);

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

std::vector<double> StructReader::requireVector3(const std::string &key) const
{
    const auto &child = getMember(key);
    const auto context = makeContext(context_, key);
    if (child.getType() != XmlRpc::XmlRpcValue::TypeArray || child.size() != 3)
    {
        throw std::runtime_error("parameter '" + context + "' must be an array of length 3");
    }
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
