/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef LOG_H
#define LOG_H

#include <sstream>
#include <string>
#include <map>
#include <stdio.h>

inline std::string NowTime();

typedef int TLogLevel;
static const TLogLevel
  logNONE    = 0,
  logERROR   = 1,
  logWARNING = 2,
  logINFO    = 3,
  logDEBUG   = 4,
  logDEBUG1  = 5,
  logDEBUG2  = 6,
  logDEBUG3  = 7,
  logDEBUG4  = 8;


class Logger
{
public:
    Logger();
    virtual ~Logger();
    std::ostringstream& Get(TLogLevel level = logINFO);
public:
    static TLogLevel& ReportingLevel();
    static std::string ToString(TLogLevel level);
    static TLogLevel FromString(const std::string& level);
protected:
    std::ostringstream os;
private:
    Logger(const Logger&);
    Logger& operator =(const Logger&);
};

inline Logger::Logger()
{
  
}

inline std::ostringstream& Logger::Get(TLogLevel level)
{
    os << "- " << NowTime();
    os << " " << ToString(level) << ": ";
    os << std::string( (level > logDEBUG) ? (level - logDEBUG) : 0, '\t');
    return os;
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
inline Logger::~Logger()
{
#ifdef LOG_TO_FILE
    os << std::endl;
    FILE * pFile;
    static int pid = getpid();
    char buffer[9];
    sprintf(buffer,"%06d",pid);
    static std::string name("out-"+std::string(buffer)+".log");
    pFile = fopen(name.c_str(),"a");
    fprintf(pFile, "%s", os.str().c_str());
    fclose (pFile);
//#else
//    fprintf(stdout, "%s", os.str().c_str());
//    fflush(stdout);
#endif
}

inline TLogLevel& Logger::ReportingLevel()
{
    static TLogLevel reportingLevel;
    return reportingLevel;
}

inline std::string Logger::ToString(TLogLevel level)
{
    static const char* const buffer[] = {"NONE","ERROR", "WARNING", "INFO", "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4"};
    return buffer[level];
}

inline TLogLevel Logger::FromString(const std::string& level)
{
    if (level == "DEBUG4")
        return logDEBUG4;
    if (level == "DEBUG3")
        return logDEBUG3;
    if (level == "DEBUG2")
        return logDEBUG2;
    if (level == "DEBUG1")
        return logDEBUG1;
    if (level == "DEBUG")
        return logDEBUG;
    if (level == "INFO")
        return logINFO;
    if (level == "WARNING")
        return logWARNING;
    if (level == "ERROR")
        return logERROR;
    if (level == "NONE")
        return logNONE;
    Logger().Get(logWARNING) << "Unknown logging level '" << level << "'. Using INFO level as default.";
    return logINFO;
}

typedef Logger FILELog;
#ifdef LOG_TO_FILE
#define OUT_LOG(level) \
    if (level > FILELog::ReportingLevel()) ; \
    else FILELog().Get(level)
#else
#define OUT_LOG(level) \
  if (1) ; \
  else FILELog().Get(level)
#endif
#define LOG(level) (level > FILELog::ReportingLevel())

#include <sys/time.h>

inline std::string NowTime()
{
    char buffer[11];
    time_t t;
    time(&t);
    tm r = {0};
    strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[100] = {0};
    sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000);
    return result;
}

extern std::string LOG_TYPE;


#endif // LOG_H
