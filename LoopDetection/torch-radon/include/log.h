#ifndef TORCH_RADON_LOG_H
#define TORCH_RADON_LOG_H

#include <sstream>
#include <string>

namespace Log
{
    enum class Level
    {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3
    };

    extern Level log_level;

    // inline Level get_log_level() { return log_level; }
    // void setGlobalLevel(Level maximumLevel);

    void log(Level level, const std::string &message, const std::string &file, const int line);
}

#define LOG_DEBUG(_message_)                                        \
    {                                                               \
        std::ostringstream oss;                                     \
        oss << _message_;                                           \
        Log::log(Log::Level::Debug, oss.str(), __FILE__, __LINE__); \
    }

#define LOG_INFO(_message_)                                        \
    {                                                              \
        std::ostringstream oss;                                    \
        oss << _message_;                                          \
        Log::log(Log::Level::Info, oss.str(), __FILE__, __LINE__); \
    }

#define LOG_WARNING(_message_)                                        \
    {                                                                 \
        std::ostringstream oss;                                       \
        oss << _message_;                                             \
        Log::log(Log::Level::Warning, oss.str(), __FILE__, __LINE__); \
    }

#define LOG_ERROR(_message_)                                        \
    {                                                               \
        std::ostringstream oss;                                     \
        oss << _message_;                                           \
        Log::log(Log::Level::Error, oss.str(), __FILE__, __LINE__); \
    }

#endif
