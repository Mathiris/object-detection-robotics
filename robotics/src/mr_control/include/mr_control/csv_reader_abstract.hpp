#pragma once

#include <iostream>
#include <string>
#include <vector>

class CSVReaderAbstract {
 public:
  CSVReaderAbstract(std::string filename, std::string delm = ",",
                    bool reverse = false)
      : fileName_(filename), delimeter_(delm) {}

  virtual std::vector<std::vector<std::string> > getData() = 0;

  virtual ~CSVReaderAbstract() {}

 protected:
  std::string fileName_;
  std::string delimeter_;
};
