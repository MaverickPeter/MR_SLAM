#include "log.h"

#include <iostream>

namespace Log {
#ifdef DEBUG
    Level log_level = Level::Debug;
#else
    Level log_level = Level::Warning;
#endif

    void log(Level level, const std::string &message, const std::string &file, const int line) {
        // TODO add colors and maybe time
        if (level >= log_level) {
            std::cout << "[Torch Radon] " << message << " ('" << file << "' at line " << line << ")" << std::endl;
        }
    }

};