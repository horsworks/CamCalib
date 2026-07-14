#include "utils/Logger.h"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace camcalib::utils {

static bool g_logEnabled = false;
static std::ofstream g_logStream;

static void logMessage(const std::string& level, const std::string& message){
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    std::ostringstream stream;
    if(localTime != nullptr){
        stream << std::put_time(localTime, "%Y-%m-%d %H:%M:%S");
    }else{
        stream << "unknown-time";
    }
    stream << " [" << level << "] " << message;

    if(level == "ERROR"){
        std::cerr << stream.str() << std::endl;
    }else{
        std::cout << stream.str() << std::endl;
    }

    if(g_logEnabled && g_logStream.is_open()){
        g_logStream << stream.str() << std::endl;
    }
}

void initializeLogger(const CaliConfig& config){
    g_logEnabled = config.log_enabled;
    if(!g_logEnabled){
        return;
    }

    const std::filesystem::path logPath = config.log_output_file;
    try{
        if(logPath.has_parent_path()){
            std::filesystem::create_directories(logPath.parent_path());
        }
        g_logStream.open(logPath, std::ios::out | std::ios::app);
    }catch(const std::filesystem::filesystem_error& e){
        std::cerr << "Failed to prepare log file: " << e.what() << std::endl;
        g_logEnabled = false;
        return;
    }

    if(!g_logStream.is_open()){
        std::cerr << "Failed to open log file: " << logPath << std::endl;
        g_logEnabled = false;
    }
}

void shutdownLogger(){
    if(g_logStream.is_open()){
        g_logStream.flush();
        g_logStream.close();
    }
    g_logEnabled = false;
}

void logInfo(const std::string& message){
    logMessage("INFO", message);
}

void logError(const std::string& message){
    logMessage("ERROR", message);
}

}  // namespace camcalib::utils
