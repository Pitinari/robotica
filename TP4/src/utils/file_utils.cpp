#include "utils/file_utils.h"
#include <fstream>
#include <iostream>
#include <sys/stat.h>

// Function to check if a file exists
bool fileExists(const std::string &filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

// Function to read a text file and return lines as vector of strings
std::vector<std::string> readTextFile(const std::string &filename)
{
    std::vector<std::string> lines;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return lines;
    }

    std::string line;
    while (std::getline(file, line))
    {
        lines.push_back(line);
    }

    file.close();
    return lines;
}