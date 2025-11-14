#include "utils/file_utils.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Function to read lines from a file and return them as a vector of strings
std::vector<std::string> readLinesFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::string> lines;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return lines;
    }

    while (std::getline(file, line)) {
        lines.push_back(line);
    }

    file.close();
    return lines;
}

// Function to write lines to a file from a vector of strings
void writeLinesToFile(const std::string& filename, const std::vector<std::string>& lines) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    for (const auto& line : lines) {
        file << line << std::endl;
    }

    file.close();
}