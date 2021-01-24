#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>

#include "mr_control/csv_reader.hpp"

std::vector<std::string> stringSplit(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

CSVReader::CSVReader(std::string filename, std::string delm, bool reverse)
    : CSVReaderAbstract(filename, delm, reverse){}


std::vector<std::vector<std::string> > CSVReader::getData() {
  std::ifstream file(fileName_);
  std::vector<std::vector<std::string> > data_list;
  if (!file.is_open()) {
    std::cout << "Error while opening file " << fileName_ << ": ";
    std::cout << std::strerror(errno) << std::endl;
  } else {
    std::string line = "";
    // Iterate through each line and split the content using delimeter
    while (getline(file, line)) {
      data_list.push_back(stringSplit(line, delimeter_[0]));
    }
    // Close the File
    file.close();
  }

  return data_list;
}

void CSVReader::PrintData(const std::vector<std::vector<std::string> > data) {
  for (auto&& line : data) {
    for (auto&& word : line) {
      std::cout << word;
    }
    std::cout << std::endl;
  }
}

CSVReader::~CSVReader() {}
