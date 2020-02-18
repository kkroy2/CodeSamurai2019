#include<bits/stdc++.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;

double pi = 4 * atan(1);
map< pair<string, string>, int > tot;

class CSVRow {
public:
	std::string const& operator[](std::size_t index) const {
		return m_data[index];
	}
	std::size_t size() const {
		return m_data.size();
	}
	void readNextRow(std::istream& str) {
		std::string line;
		std::getline(str, line);

		std::stringstream lineStream(line);
		std::string cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ',')) {
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty()) {
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}
private:
	std::vector<std::string> m_data;
};


std::istream& operator>>(std::istream& str, CSVRow& data) {
	data.readNextRow(str);
	return str;
}


int main() {

	ifstream file("../dataset/RoadmapDhaka.csv");
	CSVRow row;
    int sum =0;
	while (file >> row) {
        //cout<<row.size()<<endl;

        for(int i=1; i < row.size()-3; i+=2)
        {
            sum += 1;
            tot[make_pair(row[i], row[i+1])] += 1;
        }

        //map< pair<string, string>, int> :: iterator it= tot.begin();

        //

	}
	map< pair<string, string>, int> :: iterator it = tot.begin();
	for (std::map<pair<string, string>, int>::iterator it=tot.begin(); it!=tot.end(); ++it)
    std::cout << it->first.first <<" "<< it->first.second<< " => " << it->second << '\n';
	 cout<<tot.size()<<" "<<sum<<endl;

	return 0;
}
