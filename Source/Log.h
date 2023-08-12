//
// Created by dallin on 6/3/23.
//

#ifndef TESSELLATION_LOG_H
#define TESSELLATION_LOG_H
#include <spdlog/spdlog.h>

namespace Debug {
    class Log {
    public:
        static void Init();

        inline static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
        inline static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }

    private:
        static std::shared_ptr<spdlog::logger> s_CoreLogger;
        static std::shared_ptr<spdlog::logger> s_ClientLogger;
    };
}


#pragma mark --Logging Functions--
#define DEBUG_CORE_TRACE(...) ::Debug::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define DEBUG_CORE_INFO(...) ::Debug::Log::GetCoreLogger()->info(__VA_ARGS__)
#define DEBUG_CORE_WARN(...) ::Debug::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define DEBUG_CORE_ERROR(...) ::Debug::Log::GetCoreLogger()->error(__VA_ARGS__)
#define DEBUG_CORE_FATAL(...) ::Debug::Log::GetCoreLogger()->fatal(__VA_ARGS__)

#define DEBUG_TRACE(...) ::Debug::Log::GetClientLogger()->trace(__VA_ARGS__)
#define DEBUG_INFO(...) ::Debug::Log::GetClientLogger()->info(__VA_ARGS__)
#define DEBUG_WARN(...) ::Debug::Log::GetClientLogger()->warn(__VA_ARGS__)
#define DEBUG_ERROR(...) ::Debug::Log::GetClientLogger()->error(__VA_ARGS__)
#define DEBUG_FATAL(...) ::Debug::Log::GetClientLogger()->fatal(__VA_ARGS__)



#endif //TESSELLATION_LOG_H
