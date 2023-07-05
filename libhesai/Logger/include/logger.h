#ifndef __LOGGER_H__  
#define __LOGGER_H__  
  
#include <string>  
#include <memory>  
#include <thread>  
#include <mutex>  
#include <condition_variable>  
#include <list>  
#include <functional>
#include <algorithm>
  
//struct FILE;  

enum LOGLEVEL
{
	LOG_DEBUG   =  0x01,      /*debug*/
    LOG_INFO    =  0x02,      /*info*/
    LOG_WARNING    =  0x04,      /*warning*/
    LOG_ERROR     =  0x08,      /*error*/
    LOG_FATAL  =  0x10,      /*assert*/
};

enum LOGTARGET
{
	LOG_TARGET_NONE      = 0x00,
	LOG_TARGET_CONSOLE   = 0x01,
	LOG_TARGET_FILE      = 0x10
};

#define LogDebug(...)        Logger::GetInstance().AddToQueue(LOG_DEBUG, __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)  
#define LogInfo(...)        Logger::GetInstance().AddToQueue(LOG_INFO, __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)  
#define LogWarning(...)     Logger::GetInstance().AddToQueue(LOG_WARNING, __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)  
#define LogError(...)       Logger::GetInstance().AddToQueue(LOG_ERROR, __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define LogFatal(...)       Logger::GetInstance().AddToQueue(LOG_FATAL, __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)    

class Logger  
{  
public:  
    static Logger& GetInstance();  
  
    void SetFileName(const char* filename);  

    bool Start();  
    void Stop();  
  
    void AddToQueue(LOGLEVEL loglevel, const char* pszFile, int lineNo, const char* pszFuncSig, char* pszFmt, ...); 
	void setLogLevelRule(uint8_t rule);
	void setLogTargetRule(uint8_t rule);
	void bindLogCallback(std::function<void(LOGLEVEL loglevel, const char* pszFile, int lineNo, const char* pszFuncSig, char* pszFmt)> log_callback);

private:  
    Logger() = default;  
    Logger(const Logger& rhs) = delete;  
    Logger& operator =(Logger& rhs) = delete;  
  
    void threadfunc();  
  
  
private:  
    std::string                     filename_;  
    FILE*                           fp_{};  
    std::shared_ptr<std::thread>    spthread_;  
    std::mutex                      mutex_;  
    std::condition_variable         cv_;            //有新的日志到来的标识  
    bool                            exit_{false};  
    std::list<std::string>          queue_;  
	uint8_t                         log_level_rule_;
	uint8_t                         log_target_rule_;
	std::function<void(LOGLEVEL loglevel, const char* pszFile, int lineNo, const char* pszFuncSig, char* pszFmt)> log_callback_;
    bool                            running_{false};

};  

#endif