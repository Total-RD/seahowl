// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include <vector>
#include <string>

using namespace std;

vector<vector<double>> csv_read(const string& filename, bool hasHeader = false);
void csv_write(const string& filename, const vector<vector<double>>& data, vector<string> headers = {});
