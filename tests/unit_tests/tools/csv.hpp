#include <vector>
#include <string>

using namespace std;

vector<vector<double>> CSVRead(const string& filename, bool hasHeader = false);
void CSVWrite(const string& filename, const vector<vector<double>>& data, vector<string> headers = {});
