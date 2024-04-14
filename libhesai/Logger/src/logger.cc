/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/
#include "logger.h"  
#include <time.h>  
#include <stdio.h>  
#include <memory>  
#include <stdarg.h>  
  
Logger& Logger::GetInstance()  
{  
    static Logger logger;  
    return logger;  
}  
  
void Logger::SetFileName(const char* filename)  
{  
    filename_ = filename;  
}  
  
bool Logger::Start()  
{  

    if (running_ == true) return true;
    if (filename_.empty())  
    {  
        time_t now = time(NULL);  
        struct tm* t = localtime(&now);  
        char timestr[64] = { 0 };  
        sprintf(timestr, "%04d%02d%02d%02d%02d%02d.imserver.log", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);  
        filename_ = timestr;  
    }  
    fp_ = fopen(filename_.c_str(), "wt+");  
    if (fp_ == NULL)  
        return false;  
  
    spthread_.reset(new std::thread(std::bind(&Logger::threadfunc, this)));
    printf("logger start to run\n");
    running_ = true;  
  
    return true;  
}  
  
void Logger::Stop()  
{  
    exit_ = true;  
    cv_.notify_one();  
  
    //等待时间线程结束  
    spthread_->join();  
}  
  
void Logger::AddToQueue(LOGLEVEL loglevel, const char* pszFile, int lineNo, const char* pszFuncSig, char* pszFmt, ...)  
{ 
	
	if ((loglevel & log_level_rule_) != loglevel) return;
    char msg[256] = { 0 };  
  
    va_list vArgList;                              
    va_start(vArgList, pszFmt);  
    vsnprintf(msg, 256, pszFmt, vArgList);  
    va_end(vArgList);  
	if (log_callback_ != nullptr) {
		log_callback_(loglevel, pszFile, lineNo, pszFuncSig, msg);
		return;
	}
  
    time_t now = time(NULL);  
    struct tm* tmstr = localtime(&now);  
    char content[512] = { 0 };  
	const char* logLevel;
	if (loglevel == LOG_DEBUG){
		logLevel = "DEBUG";
	}
	else if (loglevel == LOG_INFO){
		logLevel = "INFO";
	}
	else if (loglevel == LOG_WARNING){
		logLevel = "WARNING";
	}
	else if (loglevel == LOG_ERROR){
		logLevel = "ERROR";
	}
	else if (loglevel == LOG_FATAL){
		logLevel = "FATAL";
	}
    sprintf(content, "[%04d-%02d-%02d %02d:%02d:%02d][%s][0x%04x][%s:%d %s]%s",  
                tmstr->tm_year + 1900,  
                tmstr->tm_mon + 1,  
                tmstr->tm_mday,  
                tmstr->tm_hour,  
                tmstr->tm_min,  
                tmstr->tm_sec,  
                logLevel,  
                (unsigned int)std::hash<std::thread::id>{}(std::this_thread::get_id()),
                pszFile,  
                lineNo,  
                pszFuncSig,  
                msg);  
  
    
	if (log_target_rule_ & LOG_TARGET_FILE)
	{
		{  
			std::lock_guard<std::mutex> guard(mutex_);  
			queue_.emplace_back(content);  
		}  
		
		cv_.notify_one();  
	}
	if (log_target_rule_ & LOG_TARGET_CONSOLE)
	{
		printf("%s\n", content);
	}
}  
  
void Logger::threadfunc()  
{  
    if (fp_ == NULL)  
        return;  
  
    while (!exit_)  
    {  
        //写日志  
        std::unique_lock<std::mutex> guard(mutex_);  
        while (queue_.empty())  
        {  
            if (exit_)  
                return;  
  
            cv_.wait(guard);  
        }  
  
        //写日志  
        const std::string& str = queue_.front();  
  
        fwrite((void*)str.c_str(), str.length(), 1, fp_);  
        fflush(fp_);  
        queue_.pop_front();  
    }  
}


void Logger::setLogTargetRule(uint8_t rule)
{
	log_target_rule_ = rule;
}

void Logger::setLogLevelRule(uint8_t rule)
{
    log_level_rule_ = rule;
} 

void Logger::bindLogCallback(std::function<void(LOGLEVEL loglevel, const char* pszFile, int lineNo, const char* pszFuncSig, char* pszFmt)> log_callback) {
	log_callback_ = log_callback;
}