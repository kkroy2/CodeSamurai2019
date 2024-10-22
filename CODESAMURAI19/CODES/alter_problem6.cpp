#include<bits/stdc++.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define earthRadiusKm 6371.0
using namespace std;
typedef long long ll;

double pi = 4 * atan(1);
const int maxi = 50000;
const ll restricted_min = 21600;
const ll restricted_max = 82800;
const ll period = 15 * 60;

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
std::vector< pair<pair<int, double>, pair<int, int> > > adj[maxi];
pair<int, int> p[maxi];
std::vector<pair<pair<string, string>, pair<string, string> > > edges;
std::vector<pair<pair<pair<string, string>, pair<int, int> >, ll > >  path_temp;
int ids[maxi];

vector< vector<string> > car_path;
vector< vector<string> > metro_path;
vector< vector<string> > uttara_path;
vector< vector<string> > bikolpo_path;
long long tme[maxi];
map<pair<string, string>, string> metro_name, uttara_name, bikolpo_name;

void find_path(int curr) {
	if (p[curr].first != -1)
		find_path(p[curr].first);
	pair<string, string> node = reverse_compress[curr];
	path_temp.push_back(make_pair(make_pair(make_pair(node.first, node.second), make_pair(p[curr].second, ids[curr])), tme[curr]));
}

ll extra_waiting_time(ll time) {
	ll rem = time % period;
	if (rem == 0) return 0;
	rem = period - rem;
	return rem;
}

bool satisfies_time_restriction(ll time) {
	if (time >= restricted_min && time <= restricted_max) {
		return true;
	}
	else {
		return false;
	}
}

pair<double, vector<pair<pair<pair<string, string>, pair<int, int> >, ll > > > dijkstra(int src, int dst1, int dst2, ll time) {
	//cout<<reverse_compress[src].first<<" "<<reverse_compress[src].second<<endl;
	priority_queue <pair<ll, int> > q;
	double d[maxi];
	for (int i = 0; i < maxi; i++) {
		d[i] = INT_MAX;
		ids[i] = 0;
		tme[i] = INT_MAX;
		p[i] = make_pair(-1, -1);
	}
	q.push(make_pair(-1 * time, src));
	d[src] = 0;
	tme[src] = time;
	double speed = 10.0;
	while (!q.empty()) {
		int x = q.top().second;
		int lim = adj[x].size();
		q.pop();
		//cout<<x<<endl;

		for (int i = 0; i < lim; i++) {
			int pp = adj[x][i].first.first;
			double ppw = adj[x][i].first.second;
			int type = adj[x][i].second.first;
			int id = adj[x][i].second.second;
			double cost = 1.0;

			double t = d[x] + ppw * cost;
			double time_temp = ceil((ppw * 60.0 * 60.0) / speed);
			ll cur_time = tme[x];

			if (satisfies_time_restriction(cur_time)) {
				if (type == 0) {
					cost = 20.0;
				}
				else if (type == 1) {
					cost = 5.0;
				}
				else if (type == 2 || type == 3) {
					cost = 7.0;
				}
				t = d[x] + ppw * cost;
				if (type >= 1)
					cur_time += extra_waiting_time(cur_time);
			}
			else {
				cost = 20.0;
				t = d[x] + ppw * cost;
				type = 0;
			}

			cur_time += time_temp;

			if (cur_time < tme[pp]) {
				p[pp] = make_pair(x, type);
				d[pp] = t;
				ids[pp] = id;
				tme[pp] = cur_time;
				q.push(make_pair(-1 * tme[pp], pp));
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


void print_time(ll time) {
	ll hr = time / 3600;
	ll min = (time%3600)/60;
	if(hr>=12) {
		cout<<hr<<":"<<min<<" PM";
	}
	else {
		cout<<hr<<":"<<min<<" AM";
	}
}


void print_path(vector<pair<pair<pair<string, string>, pair<int, int> >, ll > > tmp, string lat1, string lon1, string lat2, string lon2, int case_src, int case_dst) {
	std::ofstream out1("output.kml");
	string tmp1;
	int n_t = tmp.size();
	ll starting_time = tmp[0].second;
	ll ending_time = tmp[n_t-1].second;
	out1 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<Placemark>\n<name>route.kml</name>\n<LineString>\n<tessellate>1</tessellate>\n<coordinates>\n";
	cout << "Problem No: 5" << endl;
	cout << "Source: (" << lat1 << ", " << lon1 << ")" << endl;
	cout << "Destination: (" << lat2 << ", " << lon2 << ")" << endl;
	for (int i = 0; i < tmp.size() - 1; i ++) {
		string start_name = "";
		string endd_name = "";
		int type = tmp[i + 1].first.second.first;

		if (type == 1) {
			start_name = metro_name[make_pair(tmp[i].first.first.first, tmp[i].first.first.second)];
			endd_name = metro_name[make_pair(tmp[i + 1].first.first.first, tmp[i + 1].first.first.second)];
		}
		else if (type == 2) {
			start_name = bikolpo_name[make_pair(tmp[i].first.first.first, tmp[i].first.first.second)];
			endd_name = bikolpo_name[make_pair(tmp[i + 1].first.first.first, tmp[i + 1].first.first.second)];
		}
		else if (type == 3) {
			start_name = uttara_name[make_pair(tmp[i].first.first.first, tmp[i].first.first.second)];
			endd_name = uttara_name[make_pair(tmp[i + 1].first.first.first, tmp[i + 1].first.first.second)];
		}

		cout << tmp[i].first.first.first << "," << tmp[i].first.first.second << ",";
		cout << tmp[i + 1].first.first.first << "," << tmp[i + 1].first.first.second << ",";
		cout << tmp[i + 1].first.second.first << "," << tmp[i + 1].first.second.second << ",";
		cout << tmp[i + 1].second << endl;
		if (start_name.size() > 0 && endd_name.size() > 0) {
			cout << start_name << " " << endd_name << endl;
		}
		int id = tmp[i + 1].first.second.second;
		//cout << id << endl;
		if (type == 0) {
			for (int i = 0; i < car_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += car_path[id][i];
				tmp1 += ",";
				tmp1 += car_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else if (type == 1) {
			for (int i = 0; i < metro_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += metro_path[id][i];
				tmp1 += ",";
				tmp1 += metro_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else if (type == 2) {
			for (int i = 0; i < bikolpo_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += bikolpo_path[id][i];
				tmp1 += ",";
				tmp1 += bikolpo_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else {
			for (int i = 0; i < uttara_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += uttara_path[id][i];
				tmp1 += ",";
				tmp1 += uttara_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
	}
	out1 << "</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>";
	out1.close();
}

void shortest_path(string lat1, string lon1, string lat2, string lon2, ll time) {
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


	pair<double, vector<pair<pair<pair<string, string>, pair<int, int> >, ll > > > tmp1 = dijkstra(src1, dst1, dst2, time);
	pair<double, vector<pair<pair<pair<string, string>, pair<int, int> >, ll > > > tmp2 = dijkstra(src2, dst1, dst2, time);


	cout << case_src << " " << case_dst << endl;
	if (tmp1.first < tmp2.first) {
		print_path(tmp1.second, lat1, lon1, lat2, lon2, case_src, case_dst);
	}
	else {
		print_path(tmp2.second, lat1, lon1, lat2, lon2, case_src, case_dst);
	}
}

double parseTime(string start_time) {
	double time = 0;

	if (start_time[start_time.size() - 2] == 'P') {
		time += 12 * 60 * 60;
	}

	start_time.pop_back();
	start_time.pop_back();
	start_time.pop_back();

	string tmp = "";
	double hour = 0, mint = 0;

	for (int i = 0; i < start_time.find(":"); i++) {
		tmp += start_time[i];
	}

	hour = stringToDouble(tmp);
	tmp = "";

	for (int i = start_time.find(":") + 1; i < start_time.size(); i++) {
		tmp += start_time[i];
	}
	mint = stringToDouble(tmp);

	time += ((hour * 60 * 60) + (mint * 60));
	return time;
}

void test() {
	string lat1 = "90.410949";
	string lon1 = "23.847244";
	string start_time = "6:05 AM";
	double time;
	time = parseTime(start_time);
	//cout<<time<<endl;
	//string lat1 = "90.408772";
	//string lon1 = "23.844125";
	// 90.410949, 23.847244, 90.4386, 23.76202
	string lat2 = "90.4386";
	string lon2 = "23.76202";
	shortest_path(lat1, lon1, lat2, lon2, time);
}




double speed[11], per_cost[11];

struct info
{
    int est_time;
    double est_cost;
    double ac_cost;
    int ac_time;
    int ac_rem_time;
    info * par;
    int way;
    int id;
    info( int _est_time , double _est_cost, double _ac_cost, int _ac_time, int _ac_rem_time, info *_par, int _way, int _id)
    {
        est_time = _est_time;
        est_cost = _est_cost;
        ac_cost = _ac_cost;
        ac_time = _ac_time;
        ac_rem_time = _ac_rem_time;
        par = _par;
        way = _way;
        id = _id;
    }

};
struct myComparator
{
    bool operator() (info* lhs, info * rhs)
    {
        // cout<<lhs<<" "<<rhs<<endl;
        // cout<<lhs->est_cost<<" in comparator "<<rhs->est_cost<<endl;
        if(lhs->est_cost < rhs->est_cost)
            return true;
        else if(rhs->est_cost > lhs->est_cost)
            return false;
        else
        {
            return lhs->ac_rem_time > rhs->ac_rem_time;
        }
    }
};

/*bool operator < (const info& lhs, const info& rhs)
{

    if(lhs.est_cost > rhs.est_cost)
    return true;
    else if(rhs.est_cost > lhs.est_cost)
    return false;
    else
    {
        return lhs.ac_time < rhs.ac_time;
    }
    //return lhs.getAge() < rhs.getAge();/
}
*/
int get_start_time(int src_time, int mode)
{
    /////complete this function later

    if(mode == 0)
    {
        return src_time;
    }
    else if(mode == 1)
    {
        //1AM = 60*60
        int per = 5*60;
        int temp = max(src_time - 60*60, 0);
        temp = (temp-1)/per + 1;
        return temp*per + 60*60;
    }
    else if(mode == 2)
    {
        //7AM = 7*60*60
        if(src_time <= 7*60*60)
        return 7*60*60;
        int per = 20*60;
        int temp = max(src_time - 7*60*60, 0);
        temp = (temp-1)/per + 1;
        // cout<<" here print temp multiplication "<<temp<<endl;
        return temp*per + 7*60*60;
    }
    else
    {
        //6AM = 6*60*60
        int per = 10*60;
        int temp = max(src_time - 6*60*60, 0);
        temp = (temp-1)/per + 1;
        return temp*per + 6*60*60;
    }
}


double get_cost(double dis, int mode)
{
    /////complete this function later
    if(mode==0)
    {
        return dis*per_cost[0];
    }
    else if(mode == 1)
    {
        return dis*per_cost[1];
    }
    else if(mode == 2)
    {
        return dis*per_cost[2];
    }
    else
    {
        return dis*per_cost[3];
    }
}

int get_needed_time(double dis, int mode)
{
    /////complete this function later
    if(mode==0)
    {
        return ceil(dis*speed[0]);
    }
    else if(mode == 1)
    {
        return ceil(dis*speed[1]);
    }
    else if(mode == 2)
    {
        return ceil(dis*speed[2]);
    }
    else
    {
        return ceil(dis*speed[3]);
    }
}

bool available(int src_time, int mode)
{
    if(mode == 0)
    {
        return true;
    }
    else if(mode == 1)
    {
        //11PM = (11+12)*60*60
        if(src_time <= 60*60) return true;
        int last_time = (11+12)*60*60;
        int per = 5*60;
        int temp = src_time - 60*60;
        temp = (temp-1)/per + 1;
        if(temp*per + 60*60 <= last_time)
        return true;
        else
        return false;
    }
    else if(mode == 2)
    {
        //10PM = (10+12)*60*60
        if(src_time <= 7*60*60) return true;
        int last_time = (11+12)*60*60;
        int per = 20*60;
        int temp = src_time - 7*60*60;
        temp = (temp-1)/per + 1;
        if(temp*per+7*60*60 <= last_time)
        return true;
        else
        return false;
    }
    else
    {
        //11PM = 6*60*60
        if(src_time <= 6*60*60) return true;
        int last_time = (11+12)*60*60;
        int per = 10*60;
        int temp = src_time - 6*60*60;
        temp = (temp-1)/per + 1;
        if(temp*per + 6*60*60 <= last_time)
        return true;
        else
        return false;
    }
}


pair< double, long long > best_cost_so_far[maxi];

vector< pair< int , int > > path_astar_vt;
void path_a_star(info *cur)
{
    info tmp_cur = *cur;
 // cout<<tmp_cur.id<<" "<<tmp_cur.way<< " here r" <<tmp_cur.par <<endl;
    if(tmp_cur.par == nullptr)
    {
        path_astar_vt.push_back(make_pair(tmp_cur.id, tmp_cur.way));
        return;
    }
    path_a_star(cur->par);
    path_astar_vt.push_back(make_pair(tmp_cur.id, tmp_cur.way));
}

double A_star(int src, int des, int src_time, int end_time, int rem_time)
{
   // cout<<src<<" src : des "<<des<<endl;
    priority_queue< info*, vector<info*>, myComparator > pq;
    for (int i = 0; i < maxi; i++) {
		best_cost_so_far[i] = make_pair(LLONG_MAX, LLONG_MIN);
	}
	best_cost_so_far[src] = make_pair(0, LLONG_MAX);
    double so_far_cost = LLONG_MAX;

    double lat1 = stringToDouble(reverse_compress[src].first);
    double lon1 = stringToDouble(reverse_compress[src].second);

    double lat2 = stringToDouble(reverse_compress[des].first);
    double lon2 = stringToDouble(reverse_compress[des].second);

    double get_dist = getDistanceFromLatLonInKm(lat1, lon1, lat2, lon2);

    int stime = get_start_time(src_time, 0);
    //cout<<get_dist/speed[0]<<endl;
    info *cur_u =  new info(rem_time, get_dist*per_cost[0], 0, stime, rem_time, nullptr, -1, src);
    // pq.push({get_dist*par_cost[0], rem_time, rem_time - get_dist/speed[0], 0, stime, rem_time, -1, 0, src});
    pq.push(cur_u);


    ///initialization_DONE////
    // return 0;
    while(!pq.empty())
    {
        info * cur_point = pq.top();
        info cur = *cur_point;
        pq.pop();

        if(cur.id == des)
        {
            cout<<cur.id<< " "<<cur.ac_cost<<" "<<cur.par->id<<" : at the end of pq"<<endl;
            path_astar_vt.clear();
            path_a_star(cur_point);
            break;
        }
        for(int i = 0; i < adj[cur.id].size(); i++)
        {


            int v = adj[cur.id][i].first.first;
            double cur_dis = adj[cur.id][i].first.second;
            int mode = adj[cur.id][i].second.first;
            bool flag = available(cur.ac_time, mode);

            if(!flag)
            {
                cout<<"NOT AVAILABLE"<<endl;
                continue;
            }


            double lat1 = stringToDouble(reverse_compress[v].first);
            double lon1 = stringToDouble(reverse_compress[v].second);

            double lat2 = stringToDouble(reverse_compress[des].first);
            double lon2 = stringToDouble(reverse_compress[des].second);

            double get_dist = getDistanceFromLatLonInKm(lat1, lon1, lat2, lon2);

            double cur_ac_cost  = cur.ac_cost + cur_dis * (double)per_cost[mode];
            double cur_est_cost = cur_ac_cost + get_dist * per_cost[1];

            int temp_start_time = get_start_time(cur.ac_time, mode);
            int cur_ac_time = temp_start_time + get_needed_time(cur_dis, mode);

            if(get_needed_time(cur_dis, mode) + (temp_start_time - cur.ac_time) + get_needed_time(get_dist, 0) > cur.ac_rem_time)
                continue;


            int cur_ac_rem = cur.ac_rem_time - get_needed_time(cur_dis, mode) - (temp_start_time - cur.ac_time);
            int cur_est_time = cur_ac_rem - get_needed_time(get_dist, 0); // always set the best possible mode for keeping remaining time larger

            // pruning 0
            if(best_cost_so_far[v].first <= cur_ac_cost && best_cost_so_far[v].second >= cur_ac_rem)
            {
               // cout<<" pruned 0"<<endl;
                continue;
            }

            if(cur_ac_time > end_time)
            {
                continue;
            }
            if(v==des)
            {
                so_far_cost = min(so_far_cost, cur_ac_cost);
            }

            // pruning 1
            if(cur_est_time >= 0)
            {
                // pruning 2
                if(cur_est_cost < so_far_cost )
                {
                    if(best_cost_so_far[v].first > cur_ac_cost)
                    {
                        best_cost_so_far[v].first = cur_ac_cost;
                        best_cost_so_far[v].second = cur_ac_rem;
                    }
                    else if(best_cost_so_far[v].first==cur_ac_cost && best_cost_so_far[v].second < cur_ac_rem)
                    {
                        best_cost_so_far[v].second = cur_ac_rem;
                    }
                    info *tmp_v = new info(cur_est_time, cur_est_cost, cur_ac_cost, cur_ac_time, cur_ac_rem, cur_point, mode, v);
                    pq.push(tmp_v);
                }

            }


        }


    }

    reverse(path_astar_vt.begin(), path_astar_vt.end());
    for(int i = 0; i < path_astar_vt.size(); i++)
    {
//        if(path_astar_vt[i].first==des)
  //         cout<<reverse_compress[path_astar_vt[i].first].first<<","<<reverse_compress[path_astar_vt[i].first].second<<","<<0<<endl;
        cout<<reverse_compress[path_astar_vt[i].first].first<<","<<reverse_compress[path_astar_vt[i].first].second<<","<<0<<endl;
    }

   // cout<<" OKAY "<<endl;

}


pair<int, int> shortest_path_2(string lat1, string lon1, string lat2, string lon2) {
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
	return make_pair(src1, dst1);
}



void test_a_start()
{
   /* string lat1 = "90.404772";
    string lon1 = "23.855136";
    string lat2 = "90.404618";
    string lon2 = "23.855031"; */
    /*string lat1 = "90.410949";
    string lon1 = "23.847244";
    string lat2 = "90.4386";
    string lon2 = "23.76202";*/

    /*string lat1 = "90.40023";
    string lon1 = "23.87596";
    string lat2 = "90.367982";
    string lon2 = "23.835966";*/

    string lat1 = "90.36295";
    string lon1 = "23.80874";
    string lat2 = "90.40737";
    string lon2 = "23.73176";

    // srclat, srclng, destlat, destlng = , , ,
    // , , ,

	pair<int, int> ids =  shortest_path_2(lat1, lon1, lat2, lon2);
    A_star(ids.first, ids.second, 6*60*60, 9*60*60, 2*60*60);
}






int main() {

    speed[0] =  (60.0 * 60.0) / 20.0;
    speed[1] = (60.0*60.0) / 15.0;
    speed[2] = (60.0 *60.0) / 10.0;
    speed[3] = (60.0*60.0) / 12.0;

    per_cost[0] = 20.0D;
    per_cost[1] = 5.0D;
    per_cost[2] = 7.0D;
    per_cost[3] = 10.0D;

	compressId = 0;
	ifstream file("../dataset/RoadmapDhaka.csv");
    ifstream file1("../dataset/RoutemapDhakaMetroRail.csv");
    ifstream file2("../dataset/RoutemapBikolpoBus.csv");
    ifstream file3("../dataset/RoutemapUttaraBus.csv");
	CSVRow row, row1, row2, row3;
	int id;
	while (file >> row) {
		id = car_path.size();
		int n_row = row.size();
		std::vector<string> temp;

		string lat_start = row[1];
		string lon_start = row[2];

		string lat_end = row[n_row - 4];
		string lon_end = row[n_row - 3];

		if (compress[make_pair(lat_start, lon_start)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_start, lon_start)] = compressId;
			reverse_compress[compressId] = make_pair(lat_start, lon_start);
		}

		if (compress[make_pair(lat_end, lon_end)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_end, lon_end)] = compressId;
			reverse_compress[compressId] = make_pair(lat_end, lon_end);
		}

		edges.push_back(make_pair(make_pair(lat_start, lon_start), make_pair(lat_end, lon_end)));

		int u = compress[make_pair(lat_start, lon_start)];
		int v = compress[make_pair(lat_end, lon_end)];
		double w = stringToDouble(row[n_row - 1]);
		//w = w*20.0;
		//cout<<u<<" "<<v<<" "<<w<<endl;
		adj[u].push_back(make_pair(make_pair(v, w), make_pair(0, id)));

		adj[v].push_back(make_pair(make_pair(u, w), make_pair(0, id + 1)));

		for (int i = 1; i <= row.size() - 3; i++) {
			temp.push_back(row[i]);
		}

		std::vector<string> tempr;


		car_path.push_back(temp);
		reverse(temp.begin(), temp.end());

		for (int i = 0; i < temp.size() - 1; i += 2) {
			swap(temp[i], temp[i + 1]);
		}

		car_path.push_back(temp);
	}

	while (file1 >> row1) {
		id = metro_path.size();
		int n_row1 = row1.size();
		std::vector<string> temp;

		string lat_start = row1[1];
		string lon_start = row1[2];

		string lat_end = row1[n_row1 - 4];
		string lon_end = row1[n_row1 - 3];

		//edges.push_back(make_pair(make_pair(lat_start, lon_start), make_pair(lat_end, lon_end)));

		if (compress[make_pair(lat_start, lon_start)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_start, lon_start)] = compressId;
			reverse_compress[compressId] = make_pair(lat_start, lon_start);
		}

		if (compress[make_pair(lat_end, lon_end)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_end, lon_end)] = compressId;
			reverse_compress[compressId] = make_pair(lat_end, lon_end);
		}

		int u = compress[make_pair(lat_start, lon_start)];
		int v = compress[make_pair(lat_end, lon_end)];
		metro_name[make_pair(lat_start, lon_start)] = row1[n_row1 - 2];
		metro_name[make_pair(lat_end, lon_end)] = row1[n_row1 - 1];

		double w = 0;

		for (int i = 1; i < row1.size() - 5; i += 2) {
			string lat1 = (row1[i]);
			string lon1 = (row1[i + 1]);

			string lat2 = (row1[i + 2]);
			string lon2 = (row1[i + 3]);


			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			w += dis;
		}

		//w = w * 5.0;
		adj[u].push_back(make_pair(make_pair(v, w), make_pair(1, id)));
		adj[v].push_back(make_pair(make_pair(u, w), make_pair(1, id + 1)));

		for (int i = 1; i <= row1.size() - 3; i++) {
			temp.push_back(row1[i]);
		}

		metro_path.push_back(temp);
		reverse(temp.begin(), temp.end());

		for (int i = 0; i < temp.size() - 1; i += 2) {
			swap(temp[i], temp[i + 1]);
		}

		metro_path.push_back(temp);
	}

	while (file2 >> row2) {
		id = bikolpo_path.size();
		int n_row2 = row2.size();
		std::vector<string> temp;

		string lat_start = row2[1];
		string lon_start = row2[2];

		string lat_end = row2[n_row2 - 4];
		string lon_end = row2[n_row2 - 3];
		//edges.push_back(make_pair(make_pair(lat_start, lon_start), make_pair(lat_end, lon_end)));

		if (compress[make_pair(lat_start, lon_start)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_start, lon_start)] = compressId;
			reverse_compress[compressId] = make_pair(lat_start, lon_start);
		}

		if (compress[make_pair(lat_end, lon_end)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_end, lon_end)] = compressId;
			reverse_compress[compressId] = make_pair(lat_end, lon_end);
		}

		int u = compress[make_pair(lat_start, lon_start)];
		int v = compress[make_pair(lat_end, lon_end)];
		double w = 0;

		bikolpo_name[make_pair(lat_start, lon_start)] = row2[n_row2 - 2];
		bikolpo_name[make_pair(lat_end, lon_end)] = row2[n_row2 - 1];

		for (int i = 1; i < row2.size() - 5; i += 2) {
			string lat1 = (row2[i]);
			string lon1 = (row2[i + 1]);

			string lat2 = (row2[i + 2]);
			string lon2 = (row2[i + 3]);


			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			w += dis;
		}

		//w = w * 7.0;
		adj[u].push_back(make_pair(make_pair(v, w), make_pair(2, id)));
		adj[v].push_back(make_pair(make_pair(u, w), make_pair(2, id + 1)));

		for (int i = 1; i <= row2.size() - 3; i++) {
			temp.push_back(row2[i]);
		}

		bikolpo_path.push_back(temp);
		reverse(temp.begin(), temp.end());

		for (int i = 0; i < temp.size() - 1; i += 2) {
			swap(temp[i], temp[i + 1]);
		}

		bikolpo_path.push_back(temp);
	}


	while (file3 >> row3) {
		id = uttara_path.size();
		int n_row3 = row3.size();
		std::vector<string> temp;

		string lat_start = row3[1];
		string lon_start = row3[2];

		string lat_end = row3[n_row3 - 4];
		string lon_end = row3[n_row3 - 3];
		//edges.push_back(make_pair(make_pair(lat_start, lon_start), make_pair(lat_end, lon_end)));

		if (compress[make_pair(lat_start, lon_start)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_start, lon_start)] = compressId;
			reverse_compress[compressId] = make_pair(lat_start, lon_start);
		}

		if (compress[make_pair(lat_end, lon_end)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_end, lon_end)] = compressId;
			reverse_compress[compressId] = make_pair(lat_end, lon_end);
		}

		int u = compress[make_pair(lat_start, lon_start)];
		int v = compress[make_pair(lat_end, lon_end)];
		double w = 0;
		uttara_name[make_pair(lat_start, lon_start)] = row3[n_row3 - 2];
		uttara_name[make_pair(lat_end, lon_end)] = row3[n_row3 - 1];

		for (int i = 1; i < row3.size() - 5; i += 2) {
			string lat1 = (row3[i]);
			string lon1 = (row3[i + 1]);

			string lat2 = (row3[i + 2]);
			string lon2 = (row3[i + 3]);


			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			w += dis;
		}

		//w = w * 7.0;
		adj[u].push_back(make_pair(make_pair(v, w), make_pair(3, id)));
		adj[v].push_back(make_pair(make_pair(u, w), make_pair(3, id + 1)));

		for (int i = 1; i <= row3.size() - 3; i++) {
			temp.push_back(row3[i]);
		}

		uttara_path.push_back(temp);
		reverse(temp.begin(), temp.end());

		for (int i = 0; i < temp.size() - 1; i += 2) {
			swap(temp[i], temp[i + 1]);
		}

		uttara_path.push_back(temp);
	}

	string src_lat, src_lon, dst_lat, dst_lon;

    test_a_start();
	return 0;
}
