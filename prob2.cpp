#include<bits/stdc++.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define earthRadiusKm 6371.0
using namespace std;

double pi = 4 * atan(1);
const int maxi = 50000;

double deg2rad(double deg) {
	return (deg * M_PI / 180);
}

double stringToDouble(string word) {
	return atof(word.c_str());
}

pair<double, pair<double, double> > toCartesian(string lat, string lon) {
	double R = 6371;
	double x = R * cos(deg2rad(stringToDouble(lat))) * cos(deg2rad(stringToDouble(lon)));
	double y = R * cos(deg2rad(stringToDouble(lat))) * sin(deg2rad(stringToDouble(lon)));
	double z = R * sin(deg2rad(stringToDouble(lat)));
	return make_pair(x, make_pair(y, z));
}

bool onSegment(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	pair<double, pair<double, double> > tmp1 = toCartesian(lat1, lon1);
	pair<double, pair<double, double> > tmp2 = toCartesian(lat2, lon2);
	pair<double, pair<double, double> > tmpf = toCartesian(latf, lonf);

	double x1 = tmp1.first, y1 = tmp1.second.first, z1 = tmp1.second.second;
	double x2 = tmp2.first, y2 = tmp2.second.first, z2 = tmp2.second.second;
	double xf = tmpf.first, yf = tmpf.second.first, zf = tmpf.second.second;

	double max_x = max(x1, x2);
	double max_y = max(y1, y2);
	double max_z = max(z1, z2);

	double min_x = min(x1, x2);
	double min_y = min(y1, y2);
	double min_z = min(z1, z2);

	if ((xf >= min_x && xf <= max_x) && (yf >= min_y && yf <= max_y) && (zf >= min_z && zf <= max_z))
		return true;
	return false;
}

double orientation(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	double ax = stringToDouble(lat1);
	double ay = stringToDouble(lon1);
	double bx = stringToDouble(lat2);
	double by = stringToDouble(lon2);
	double cx = stringToDouble(latf);
	double cy = stringToDouble(lonf);
	return abs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
}

bool onSegmentLatLon(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	double x1 = stringToDouble(lat1);
	double y1 = stringToDouble(lon1);
	double x2 = stringToDouble(lat2);
	double y2 = stringToDouble(lon2);
	double xf = stringToDouble(latf);
	double yf = stringToDouble(lonf);

	double max_x = max(x1, x2);
	double max_y = max(y1, y2);

	double min_x = min(x1, x2);
	double min_y = min(y1, y2);

	if ((xf >= min_x && xf <= max_x) && (yf >= min_y && yf <= max_y)) {
		//cout<<orientation(lat1, lon1, lat2, lon2, latf, lonf)<< " return value"<<endl;
		if (orientation(lat1, lon1, lat2, lon2, latf, lonf) <= 0.00000001) return true;
		return false;
	}
	return false;
}

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

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
	return (rad * 180 / M_PI);
}

double getDistanceFromLatLonInKm(double lat1d, double lon1d, double lat2d, double lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	swap(lat1d, lon1d);
	swap(lat2d, lon2d);
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double getDistance(double lat1, double long1, double lat2, double long2) {
	double ret = sqrt((lat1 - lat2) * (lat1 - lat2) + (long1 - long2) * (long1 - long2));
	return ret;
}

std::map<pair<string, string>, int> compress;
std::map<int, pair<string, string> > reverse_compress;
int compressId;
std::vector< pair<pair<int, double>, int> > adj[maxi];
pair<int, int> p[maxi];
std::vector<pair<pair<string, string>, pair<string, string> > > edges;
std::vector<pair<pair<string, string>, int > >  path_temp;

void find_path(int curr) {
	if (p[curr].first != -1)
		find_path(p[curr].first);
	pair<string, string> node = reverse_compress[curr];
	path_temp.push_back(make_pair(make_pair(node.first, node.second), p[curr].second));
}

pair<double, vector<pair<pair<string, string>, int > > > dijkstra(int src, int dst1, int dst2) {
	//cout<<reverse_compress[src].first<<" "<<reverse_compress[src].second<<endl;
	priority_queue <pair<double, int> > q;
	double d[maxi];
	for (int i = 0; i < maxi; i++) {
		d[i] = INT_MAX;
		p[i] = make_pair(-1, -1);
	}
	q.push(make_pair(0, src));
	d[src] = 0;

	while (!q.empty()) {
		int x = q.top().second;
		int lim = adj[x].size();
		q.pop();
		//cout<<x<<endl;

		for (int i = 0; i < lim; i++) {
			int pp = adj[x][i].first.first;
			double ppw = adj[x][i].first.second;
			int type = adj[x][i].second;
			double cost = 1.0;

			if (type == 0) {
				cost = 20.0;
			}
			else if (type == 1) {
				cost = 5.0;
			}
			else if (type == 2 || type == 3) {
				cost = 7.0;
			}

			double t = d[x] + ppw;


			if (t < d[pp]) {
				p[pp] = make_pair(x, type);
				d[pp] = t;
				q.push(make_pair(-1.0 * d[pp], pp));
			}
		}
	}

	path_temp.clear();

	if (d[dst1] < d[dst2])
		find_path(dst1);
	else
		find_path(dst2);

	return make_pair(min(d[dst1], d[dst2]), path_temp);
}

void print_path(vector<pair<pair<string, string>, int > > tmp, string lat1, string lon1, string lat2, string lon2) {
	cout << lat1 << " " << lon1 << endl;
	cout << lat2 << " " << lon2 << endl;
	for (int i = 0; i < tmp.size(); i++) {
		cout << tmp[i].first.first << "," << tmp[i].first.second << "," << 0 << "," << tmp[i].second << endl;
	}
}

void shortest_path(string lat1, string lon1, string lat2, string lon2) {
	int src1, src2, dst1, dst2;
	int case_src = -1, case_dst = -1;

	if (compress.find(make_pair(lat1, lon1)) != compress.end()) {
		src1 = compress[make_pair(lat1, lon1)];
		src2 = src1;
		case_src = 1;
	}
	else {
		bool ok = false;

		for (int i = 0; i < edges.size(); i++) {
			string latt1 = edges[i].first.first;
			string lont1 = edges[i].first.second;
			string latt2 = edges[i].second.first;
			string lont2 = edges[i].second.second;
			if (onSegmentLatLon(latt1, lont1, latt2, lont2, lat1, lon1)) {
				src1 = compress[make_pair(latt1, lont1)];
				src2 = compress[make_pair(latt2, lont2)];
				cout << latt1 << " " << lont1 << " " << latt2 << " " << lont2 << endl;
				ok = true;
				case_src = 2;
				break;
			}
		}

		if (!ok) {
			double minDis = DBL_MAX, minDisId = -1;
			for (int i = 1; i <= compressId; i++) {
				double lat = stringToDouble(reverse_compress[i].first);
				double lon = stringToDouble(reverse_compress[i].second);

				double diss = getDistanceFromLatLonInKm(lat, lon, stringToDouble(lat1), stringToDouble(lon1));
				if (diss < minDis) {
					minDis = diss;
					minDisId = i;
				}
			}
			case_src = 3;
			src1 = src2 = minDisId;
		}
	}


	if (compress.find(make_pair(lat2, lon2)) != compress.end()) {
		dst1 = compress[make_pair(lat2, lon2)];
		dst2 = dst1;
		case_dst = 1;
	}
	else {
		bool ok = false;

		for (int i = 0; i < edges.size(); i++) {
			string latt1 = edges[i].first.first;
			string lont1 = edges[i].first.second;
			string latt2 = edges[i].second.first;
			string lont2 = edges[i].second.second;
			if (onSegmentLatLon(latt1, lont1, latt2, lont2, lat2, lon2)) {
				dst1 = compress[make_pair(latt1, lont1)];
				dst2 = compress[make_pair(latt2, lont2)];
				ok = true;
				case_dst = 2;
				break;
			}
		}

		if (!ok) {
			double minDis = DBL_MAX, minDisId = -1;
			for (int i = 1; i <= compressId; i++) {
				double lat = stringToDouble(reverse_compress[i].first);
				double lon = stringToDouble(reverse_compress[i].second);

				double diss = getDistanceFromLatLonInKm(lat, lon, stringToDouble(lat2), stringToDouble(lon2));
				if (diss < minDis) {
					minDis = diss;
					minDisId = i;
				}
			}
			case_dst = 3;
			dst1 = dst2 = minDisId;
		}
	}


	pair<double, vector<pair<pair<string, string>, int > > > tmp1 = dijkstra(src1, dst1, dst2);
	pair<double, vector<pair<pair<string, string>, int > > > tmp2 = dijkstra(src2, dst1, dst2);


	cout << case_src << " " << case_dst << endl;
	if (tmp1.first < tmp2.first) {
		print_path(tmp1.second, lat1, lon1, lat2, lon2);
	}
	else {
		print_path(tmp2.second, lat1, lon1, lat2, lon2);
	}
}

void test() {
	//string lat1 = "90.404772";
	//string lon1 = "23.855136";
	string lat1 = "90.408772";
	string lon1 = "23.844125";
	string lat2 = "90.4386";
	string lon2 = "23.76202";
	shortest_path(lat1, lon1, lat2, lon2);
}


int main() {
	compressId = 0;
	ifstream file("Code Samurai 2019 - Problem Resources/Roadmap-Dhaka.csv");
	ifstream file1("Code Samurai 2019 - Problem Resources/Routemap-DhakaMetroRail.csv");
	CSVRow row, row1, row2, row3;

	while (file >> row) {

		for (int i = 1; i < row.size() - 3; i += 2) {
			string lat = (row[i]);
			string lon = (row[i + 1]);
			if (compress[make_pair(lat, lon)] == 0) {
				compressId = compressId + 1;
				compress[make_pair(lat, lon)] = compressId;
				reverse_compress[compressId] = make_pair(lat, lon);
			}
		}

		for (int i = 1; i < row.size() - 3; i += 2) {
			string lat1 = (row[i]);
			string lon1 = (row[i + 1]);

			string lat2 = (row[i + 2]);
			string lon2 = (row[i + 3]);
			edges.push_back(make_pair(make_pair(lat1, lon1), make_pair(lat2, lon2)));

			int u = compress[make_pair(lat1, lon1)];
			int v = compress[make_pair(lat2, lon2)];

			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			dis = dis * 20.0;
			adj[u].push_back(make_pair(make_pair(v, dis), 0));
			adj[v].push_back(make_pair(make_pair(u, dis), 0));
		}

	}


	while (file1 >> row1) {

		for (int i = 1; i < row1.size() - 3; i += 2) {
			string lat = (row1[i]);
			string lon = (row1[i + 1]);
			if (compress[make_pair(lat, lon)] == 0) {
				compressId = compressId + 1;
				compress[make_pair(lat, lon)] = compressId;
				reverse_compress[compressId] = make_pair(lat, lon);
			}
		}

		for (int i = 1; i < row1.size() - 3; i += 2) {
			string lat1 = (row1[i]);
			string lon1 = (row1[i + 1]);

			string lat2 = (row1[i + 2]);
			string lon2 = (row1[i + 3]);

			int u = compress[make_pair(lat1, lon1)];
			int v = compress[make_pair(lat2, lon2)];

			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			dis = dis * 5.0;
			adj[u].push_back(make_pair(make_pair(v, dis), 1));
			adj[v].push_back(make_pair(make_pair(u, dis), 1));
		}

	}


	string src_lat, src_lon, dst_lat, dst_lon;
	//cin >> src_lat >> src_lon >> dst_lat >> dst_lon;
	//shortest_path(src_lat, src_lon, dst_lat, dst_lon);

	test();
	//cout << compressId << endl;
	//cout << "complete" << endl;
	return 0;
}