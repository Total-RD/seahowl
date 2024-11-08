#include <vector>
#include <string>

using namespace std;

vector<vector<double>> csv_read(const string& filename, bool hasHeader = false);
void csv_write(const string& filename, const vector<vector<double>>& data, vector<string> headers = {});
