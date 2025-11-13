#pragma once

#include <ros/node_handle.h>
#include <XmlRpcValue.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mag_core_utils::param
{

class StructReader;
class ArrayReader;

class StructReader
{
public:
    static StructReader fromParameter(const ros::NodeHandle &nh,
                                      const std::string &key,
                                      const std::string &context_prefix = "~");

    StructReader childStruct(const std::string &key) const;
    ArrayReader childArray(const std::string &key) const;

    bool has(const std::string &key) const;

    std::string requireString(const std::string &key) const;
    std::string optionalString(const std::string &key, const std::string &default_value) const;

    double requireNumber(const std::string &key) const;
    double optionalNumber(const std::string &key, double default_value) const;

    int requireInt(const std::string &key) const;

    std::vector<double> requireVector3(const std::string &key) const;

    const XmlRpc::XmlRpcValue &raw() const { return *value_; }
    const std::string &context() const { return context_; }

private:
    StructReader(std::shared_ptr<XmlRpc::XmlRpcValue> storage,
                 const XmlRpc::XmlRpcValue *value,
                 std::string context);

    friend class ArrayReader;

    const XmlRpc::XmlRpcValue &getMember(const std::string &key) const;

    std::shared_ptr<XmlRpc::XmlRpcValue> storage_;
    const XmlRpc::XmlRpcValue *value_{nullptr};
    std::string context_;
};

class ArrayReader
{
public:
    ArrayReader(std::shared_ptr<XmlRpc::XmlRpcValue> storage,
                const XmlRpc::XmlRpcValue *value,
                std::string context);

    int size() const;

    StructReader structAt(int index) const;

    const XmlRpc::XmlRpcValue &rawAt(int index) const;
    std::vector<double> vector3At(int index) const;
    double numberAt(int index) const;

    std::string elementContext(int index) const;

private:
    void requireIndexInRange(int index) const;

    std::shared_ptr<XmlRpc::XmlRpcValue> storage_;
    const XmlRpc::XmlRpcValue *value_{nullptr};
    std::string context_;
};

} // namespace mag_core_utils::param
