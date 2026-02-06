#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>

namespace muse_offline {

inline std::vector<std::string> splitCSV(const std::string& line) {
    std::vector<std::string> out;
    std::string token;
    std::stringstream ss(line);
    while (std::getline(ss, token, ',')) out.push_back(token);
    return out;
}

inline std::unordered_map<std::string,int> makeIndex(const std::vector<std::string>& header) {
    std::unordered_map<std::string,int> idx;
    idx.reserve(header.size());
    for (int i = 0; i < (int)header.size(); ++i) idx[header[i]] = i;
    return idx;
}

inline double getD(const std::vector<std::string>& row,
                   const std::unordered_map<std::string,int>& idx,
                   const std::string& name) {
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("Missing column: " + name);
    const int j = it->second;
    if (j < 0 || j >= (int)row.size()) throw std::runtime_error("Bad index for: " + name);
    return std::stod(row[j]);
}

class CsvReader {
public:
  explicit CsvReader(const std::string& path) : in_(path) {
    if (!in_.is_open()) throw std::runtime_error("Cannot open: " + path);
    std::string headerLine;
    if (!std::getline(in_, headerLine)) throw std::runtime_error("Empty CSV: " + path);
    header_ = splitCSV(headerLine);
    idx_ = makeIndex(header_);
  }

  bool next() {
    line_.clear();
    while (std::getline(in_, line_)) {
      if (!line_.empty()) {
        row_ = splitCSV(line_);
        return true;
      }
    }
    return false;
  }

  const std::unordered_map<std::string,int>& idx() const { return idx_; }
  const std::vector<std::string>& row() const { return row_; }

private:
  std::ifstream in_;
  std::string line_;
  std::vector<std::string> header_;
  std::unordered_map<std::string,int> idx_;
  std::vector<std::string> row_;
};

} // namespace muse_offline
