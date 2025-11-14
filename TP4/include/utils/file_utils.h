#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <string>
#include <vector>

// Function to read a G2O file and extract poses and edges
bool readG2OFile(const std::string& filename, std::vector<std::vector<double>>& poses, std::vector<std::vector<double>>& edges);

// Function to write optimized poses to a file
bool writeOptimizedPoses(const std::string& filename, const std::vector<std::vector<double>>& optimizedPoses);

// Function to read data from a text file
std::vector<std::string> readTextFile(const std::string& filename);

// Function to check if a file exists
bool fileExists(const std::string& filename);

#endif // FILE_UTILS_H