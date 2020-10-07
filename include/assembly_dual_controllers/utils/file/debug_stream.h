#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>

class DebugStream
{
public:
  DebugStream(const std::string & file_name)
  : debug_file_ (file_name) 
  {
    thread_ = std::thread(&DebugStream::worker, this);
  }

  virtual ~DebugStream()
  {
    working_ = false;
    thread_.join();
  }

  // std::stringstream& operator () ()
  // {
  //   return buffer_;
  // }
  void write (const std::string& str)
  {
    std::scoped_lock<std::mutex> lock(buffer_mutex_);
    buffer_ << str;
  }
  

private:
  void worker()
  {
    while (working_)
    {
      {
        std::scoped_lock<std::mutex> lock(buffer_mutex_);

        if (!buffer_.eof())
        {
          buffer_in_writing_ << buffer_.str();
          buffer_.clear();
        }
      }
      if (!buffer_in_writing_.eof())
      {
        debug_file_ << buffer_in_writing_.str();
        buffer_in_writing_.clear();
      }
    }
  }

  bool working_ {true};
  std::mutex buffer_mutex_;
  std::stringstream buffer_;
  std::stringstream buffer_in_writing_;
  std::ofstream debug_file_;
  std::thread thread_; 
};