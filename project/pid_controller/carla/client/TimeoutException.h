#pragma once
#include <stdexcept>
namespace carla { namespace client {
class TimeoutException : public std::exception {};
}} // namespace carla::client
