#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP

#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// getting the current time is usefull to create files with different names
std::string currentDateTime() {
  time_t now = time(nullptr);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y_%m_%d_%H_%M_%S", &tstruct);
  return buf;
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 16) {
  std::ostringstream out;
  out.precision(n);
  out << a_value;
  return out.str();
}

// taken from
// https://gist.github.com/rudolfovich/f250900f1a833e715260a66c87369d15

class CSVWriter {
 public:
  CSVWriter(const std::string filename, const std::string delm = ",")
      : fs_(), fileName_(filename), delimeter_(delm), is_first_(true) {
    fs_.exceptions(std::ios::failbit | std::ios::badbit);
    fs_.open(fileName_);
    fs_.precision(16);
  }
  ~CSVWriter() {
    flush();
    fs_.close();
  }
  void flush() { fs_.flush(); }
  void endrow() {
    fs_ << std::endl;
    is_first_ = true;
  }

  CSVWriter& operator<<(CSVWriter& f(CSVWriter&)) { return f(*this); }
  template <typename T>
  CSVWriter& operator<<(const T& val) {
    return write(val);
  }

  template <typename T>
  CSVWriter& write(const T& val) {
    if (!is_first_) {
      fs_ << delimeter_;
    } else {
      is_first_ = false;
    }
    fs_ << val;
    return *this;
  }

 private:
  std::ofstream fs_;
  std::string fileName_;
  std::string delimeter_;
  bool is_first_;
};

inline static CSVWriter& endrow(CSVWriter& file) {
  file.endrow();
  return file;
}

inline static CSVWriter& flush(CSVWriter& file) {
  file.flush();
  return file;
}

#endif  // CSV_WRITER_HPP
