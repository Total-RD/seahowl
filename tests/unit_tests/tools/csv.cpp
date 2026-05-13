// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "csv.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <spdlog/spdlog.h>

using namespace std;

vector<vector<double>> csv_read(const string& filename, bool hasHeader) {
    vector<vector<double>> data;  // Variable to store all the rows of data
    ifstream file(filename);

    // Check if the file is open
    if (!file.is_open()) {
        spdlog::error("Error opening file: " + filename);
        return data;  // Return empty data if the file can't be opened
    }

    string line;

    if (hasHeader) {
        getline(file, line);  // Skip the header
    }

    // Read the file line by line
    while (getline(file, line)) {
        stringstream ss(line);
        string value;
        vector<double> row;

        while (getline(ss, value, ',')) {
            try {
                row.push_back(stod(value));
            } catch (const invalid_argument& e) {
                cerr << "Invalid value encountered: " << value << endl;
            }
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

void csv_write(const string& filename, const vector<vector<double>>& data, vector<string> headers) {
    ofstream file(filename);

    if (!file.is_open()) {
        spdlog::error("Could not open the file!");
        return;
    }

    if (!headers.empty()) {
        for (size_t i = 0; i < headers.size(); ++i) {
            file << headers[i];
            if (i < headers.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << std::setprecision(10) << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
}
