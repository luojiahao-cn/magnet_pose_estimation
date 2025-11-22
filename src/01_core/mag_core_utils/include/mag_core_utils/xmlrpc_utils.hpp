#pragma once

#include <XmlRpcValue.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mag_core_utils::xmlrpc {

std::string makeContext(const std::string &context, const std::string &member);

const XmlRpc::XmlRpcValue &requireMember(const XmlRpc::XmlRpcValue &node,
                                         const std::string &member,
                                         const std::string &context);

bool hasMember(const XmlRpc::XmlRpcValue &node, const std::string &member);

const XmlRpc::XmlRpcValue &requireStructField(const XmlRpc::XmlRpcValue &node,
                                              const std::string &member,
                                              const std::string &context);

const XmlRpc::XmlRpcValue &requireArrayField(const XmlRpc::XmlRpcValue &node,
                                             const std::string &member,
                                             const std::string &context);

std::string requireStringField(const XmlRpc::XmlRpcValue &node,
                               const std::string &member,
                               const std::string &context);

std::string optionalStringField(const XmlRpc::XmlRpcValue &node,
                                const std::string &member,
                                const std::string &context,
                                const std::string &default_value);

double optionalNumberField(const XmlRpc::XmlRpcValue &node,
                           const std::string &member,
                           const std::string &context,
                           double default_value);

bool optionalBoolField(const XmlRpc::XmlRpcValue &node,
                       const std::string &member,
                       const std::string &context,
                       bool default_value);

bool requireBoolField(const XmlRpc::XmlRpcValue &node,
                      const std::string &member,
                      const std::string &context);

std::vector<double> requireVector3Field(const XmlRpc::XmlRpcValue &node,
                                        const std::string &member,
                                        const std::string &context);

std::vector<double> requireVector9Field(const XmlRpc::XmlRpcValue &node,
                                        const std::string &member,
                                        const std::string &context);

std::vector<double> readVector3(const XmlRpc::XmlRpcValue &value,
                                const std::string &context);

std::vector<double> readVector9(const XmlRpc::XmlRpcValue &value,
                                const std::string &context);

Eigen::Vector3d readEigenVector3(const XmlRpc::XmlRpcValue &value,
                                  const std::string &context);

Eigen::Matrix3d readEigenMatrix3x3(const XmlRpc::XmlRpcValue &value,
                                    const std::string &context);

double readNumber(const XmlRpc::XmlRpcValue &value,
                  const std::string &context);

const XmlRpc::XmlRpcValue &asStruct(const XmlRpc::XmlRpcValue &value,
                                    const std::string &context);

}  // namespace mag_core_utils::xmlrpc
