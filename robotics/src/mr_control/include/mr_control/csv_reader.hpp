#include <iostream>
#include <string>
#include <vector>
#include "mr_control/csv_reader_abstract.hpp"

std::vector<std::string> stringSplit(const std::string& s, char delimiter);

class CSVReader : public CSVReaderAbstract {
 public:
  CSVReader(std::string filename, std::string delm = ",", bool reverse = false);

  std::vector<std::vector<std::string> > getData();

  static void PrintData(const std::vector<std::vector<std::string> > data);

  ~CSVReader();
};
