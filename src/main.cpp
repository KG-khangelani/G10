#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Reads all lines from the given file path into a vector of strings.
// Returns true on success, false if the file could not be opened.
bool readStateFile(const std::string &filePath, std::vector<std::string> &lines) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file '" << filePath << "'\n";
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    return true;
}

// Processes the lines read from the state file.
// Extend this function to parse the game state and decide on an action.
void processState(const std::vector<std::string> &lines) {
    std::cout << "--- State file contents (" << lines.size() << " line(s)) ---\n";
    for (const auto &line : lines) {
        std::cout << line << "\n";
    }
    std::cout << "-------------------------------------------\n";

    // TODO: parse the state and implement bot logic here
}

int main(int argc, char *argv[]) {
    // Default state file path; override via the first command-line argument.
    std::string stateFilePath = "state.txt";
    if (argc > 1) {
        stateFilePath = argv[1];
    }

    std::vector<std::string> lines;
    if (!readStateFile(stateFilePath, lines)) {
        return 1;
    }

    processState(lines);

    return 0;
}
