#include<bits/stdc++.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;

double pi = 4 * atan(1);
const int maxi = 50000;

double deg2rad(double deg) {
	return deg * (pi / 180.0);
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
    return abs(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by));
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
		if(orientation(lat1, lon1, lat2, lon2, latf, lonf) <= 0.00000001) return true;
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

double getDistanceFromLatLonInKm(double lat1, double long1, double lat2, double long2) {
    swap(lat1, long1);
    swap(lat2, long2);
	double PI = 4.0 * atan(1.0);
	double dlat1 = lat1 * (PI / 180);

	double dlong1 = long1 * (PI / 180);
	double dlat2 = lat2 * (PI / 180);
	double dlong2 = long2 * (PI / 180);

	double dLong = dlong1 - dlong2;
	double dLat = dlat1 - dlat2;

	double aHarv = pow(sin(dLat / 2.0), 2.0) + cos(dlat1) * cos(dlat2) * pow(sin(dLong / 2), 2);
	double cHarv = 2 * atan2(sqrt(aHarv), sqrt(1.0 - aHarv));
	const double earth = 6371;
	double distance = earth * cHarv;
	return distance;
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
				cout<<latt1<<" "<<lont1<<" "<<latt2<<" "<<lont2<<endl;
				ok = true;
				case_src = 2;
				break;
			}
		}

		if (!ok) {
			double minDis = INT_MAX, minDisId = -1;
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
			double minDis = INT_MAX, minDisId = -1;
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
	cout<<tmp1.first<<" cost "<<endl;
		print_path(tmp1.second, lat1, lon1, lat2, lon2);
	}
	else {
        cout<<tmp2.first<<" cost "<<endl;
		print_path(tmp2.second, lat1, lon1, lat2, lon2);
	}
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
            return lhs->ac_time < rhs->ac_time;
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
        cout<<" here print temp multiplication "<<temp<<endl;
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
  cout<<tmp_cur.id<<" "<<tmp_cur.way<< " here r" <<tmp_cur.par <<endl;
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
    // cout<<src<<" src: des "<<des<<endl;
    priority_queue< info*, vector<info*>, myComparator > pq;
    for (int i = 0; i < maxi; i++) {
		best_cost_so_far[i] = make_pair(LLONG_MAX, LLONG_MIN);
	}
	best_cost_so_far[src] = make_pair(LLONG_MIN, LLONG_MAX);

    double so_far_cost = LLONG_MAX;
   //  cout<<" MAXIMUM "<<so_far_cost<<endl;

    double lat1 = stringToDouble(reverse_compress[src].first);
    double lon1 = stringToDouble(reverse_compress[src].second);
    double lat2 = stringToDouble(reverse_compress[des].first);
    double lon2 = stringToDouble(reverse_compress[des].second);
    double get_dist = getDistanceFromLatLonInKm(lat1, lon1, lat2, lon2);
    int stime = get_start_time(src_time, 0);
    //cout<<get_dist/speed[0]<<endl;
    info *cur_u =  new info(rem_time - ceil(get_dist/speed[0]), get_dist*per_cost[0], 0, stime, rem_time, nullptr, -1, src);
    // pq.push({get_dist*par_cost[0], rem_time, rem_time - get_dist/speed[0], 0, stime, rem_time, -1, 0, src});
    pq.push(cur_u);


    ///initialization_DONE////
    // return 0;
    while(!pq.empty())
    {
        info * cur_point = pq.top();
      //  cout<<pq.size()<<endl;
        //cout<<cur<<" herer current point"<<endl;
        info cur = *cur_point;
        //cout<<cur.par<<" herer current parent point"<<endl;
      cout<<cur.est_cost<<" "<<cur.est_time<<" ID: "<<cur.id<<endl;
        pq.pop();

     //  cout<<cur.id<<" "<<cur.ac_rem_time<<endl;
        // later check the condition
     // cout<<cur.id<<" "<<cur.ac_cost<<" "<<typeid(cur.ac_cost).name()<<" "<<cur.ac_time<<" "<<cur.ac_rem_time<<" "<<cur.par<<endl;
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
            // cout<<v<<" itermediate id"<<endl;// check later
            double cur_dis = adj[cur.id][i].first.second;
            int mode = adj[cur.id][i].second;
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

            // cout<<get_dist<<"  in mid print in   dnsdsjdsnjsn"<<per_cost[mode]<<" "<<get_dist*per_cost[mode]<<endl;

            double cur_ac_cost  = cur.ac_cost + cur_dis* (double)per_cost[mode];
            double cur_est_cost = cur_ac_cost + get_dist*per_cost[mode];

            int temp_start_time = get_start_time(cur.ac_time, mode);
            int cur_ac_time = temp_start_time + get_needed_time(cur_dis, mode);
            if(get_needed_time(cur_dis, mode) + (temp_start_time - cur.ac_time) > cur.ac_rem_time)
                continue;

            int cur_ac_rem = cur.ac_rem_time - get_needed_time(cur_dis, mode) - (temp_start_time - cur.ac_time);
            int cur_est_time = cur_ac_rem - get_needed_time(get_dist, 0); // always set the best possible mode for keeping remaining time larger

            // pruning 0
            if(best_cost_so_far[v].first <= cur_ac_cost && best_cost_so_far[v].second >= cur_ac_rem)
            {
               // cout<<" pruned 0"<<endl;
                continue;
            }
           // cout<<cur.ac_cost<<" "<<cur_ac_cost<<" fucked up "<<cur.est_cost<<" "<<cur_est_cost<<endl;
          //  cout<<best_cost_so_far[v].first<<" "<<cur_ac_cost<<" "<<best_cost_so_far[v].second<<" "<<cur_ac_rem<<" "<<cur_est_time<<endl;
            if(cur_ac_time > end_time)
            {
                continue;
            }
            if(v==des)
            {

                so_far_cost = min(so_far_cost, cur_ac_cost);
            //    cout<<" destionation "<<" "<<cur_ac_cost<<endl;
                // continue;
            }



            // pruning 1


            if(cur_est_time > 0)
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
                }
               //  cout<<"caling "<<v << cur_est_time<<" "<<cur_est_cost<<endl;
               //  cout<<" before new node: "<<cur_point<<endl;
                info *tmp_v = new info(cur_est_time, cur_est_cost, cur_ac_cost, cur_ac_time, cur_ac_rem, cur_point, mode, v);
                // cout<<&cur<<" "<<&tmp_v<<endl;
              // cout<<&tmp_v<<" new point "<<endl;
                pq.push(tmp_v);
            }


        }


    }

    cout<<"DONE A_Star"<<endl;
    reverse(path_astar_vt.begin(), path_astar_vt.end());
    for(int i = 0; i < path_astar_vt.size(); i++)
    {
        if(path_astar_vt[i].first==2)
            cout<<"EXIST"<<endl;
        cout<<reverse_compress[path_astar_vt[i].first].first<<","<<reverse_compress[path_astar_vt[i].first].second<<","<<0<<endl;
    }

    cout<<" OKAY "<<endl;

}


void test() {
	//string lat1 = "90.404772";
	//string lon1 = "23.855136";
	string lat1 = "90.404772";
	string lon1 = "23.855136";
	string lat2 = "90.439188";
	string lon2 = "23.760343";
	shortest_path(lat1, lon1, lat2, lon2);
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

    string lat1 = "90.40023";
    string lon1 = "23.87596";
    string lat2 = "90.367982";
    string lon2 = "23.835966";
    // srclat, srclng, destlat, destlng = , , ,


	//shortest_path(lat1, lon1, lat2, lon2);
    A_star(compress[make_pair(lat1, lon1)], compress[make_pair(lat2, lon2)], 6*60*60, 21*60*60, 15*60*60);
}





int main() {
	compressId = 0;
	ifstream file("../dataset/RoadmapDhaka.csv");
    ifstream file1("../dataset/RoutemapDhakaMetroRail.csv");
    ifstream file2("../dataset/RoutemapBikolpoBus.csv");
    ifstream file3("../dataset/RoutemapUttaraBus.csv");
	CSVRow row, row1, row2, row3;
    speed[0] =  (60.0 * 60.0) / 20.0;
    speed[1] = (60.0*60.0) / 15.0;
    speed[2] = (60.0 *60.0) / 10.0;
    speed[3] = (60.0*60.0) / 12.0;

    per_cost[0] = 20.0D;
    per_cost[1] = 5.0D;
    per_cost[2] = 7.0D;
    per_cost[3] = 10.0D;

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
			//dis = dis;
			if(u!=v)
			{
			adj[u].push_back(make_pair(make_pair(v, dis), 0));
			adj[v].push_back(make_pair(make_pair(u, dis), 0));

			}
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
			//dis = dis;
			if(u!=v)
			{
			adj[u].push_back(make_pair(make_pair(v, dis), 1));
			adj[v].push_back(make_pair(make_pair(u, dis), 1));

			}
		}

	}

	while (file2 >> row2) {

		for (int i = 1; i < row2.size() - 3; i += 2) {
			string lat = (row2[i]);
			string lon = (row2[i + 1]);
			if (compress[make_pair(lat, lon)] == 0) {
				compressId = compressId + 1;
				compress[make_pair(lat, lon)] = compressId;
				reverse_compress[compressId] = make_pair(lat, lon);
			}
		}

		for (int i = 1; i < row2.size() - 3; i += 2) {
			string lat1 = (row2[i]);
			string lon1 = (row2[i + 1]);

			string lat2 = (row2[i + 2]);
			string lon2 = (row2[i + 3]);

			int u = compress[make_pair(lat1, lon1)];
			int v = compress[make_pair(lat2, lon2)];

			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			//dis = dis;
			if(u!=v)
			{
			adj[u].push_back(make_pair(make_pair(v, dis),2));
			adj[v].push_back(make_pair(make_pair(u, dis), 2));

			}
		}

	}


	while (file3 >> row3) {

		for (int i = 1; i < row3.size() - 3; i += 2) {
			string lat = (row3[i]);
			string lon = (row3[i + 1]);
			if (compress[make_pair(lat, lon)] == 0) {
				compressId = compressId + 1;
				compress[make_pair(lat, lon)] = compressId;
				reverse_compress[compressId] = make_pair(lat, lon);
			}
		}

		for (int i = 1; i < row3.size() - 3; i += 2) {
			string lat1 = (row3[i]);
			string lon1 = (row3[i + 1]);

			string lat2 = (row3[i + 2]);
			string lon2 = (row3[i + 3]);

			int u = compress[make_pair(lat1, lon1)];
			int v = compress[make_pair(lat2, lon2)];

			double dis = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(lat2), stringToDouble(lon2));
			//dis = dis;
			if(u!=v)
			{
			adj[u].push_back(make_pair(make_pair(v, dis), 3));
			adj[v].push_back(make_pair(make_pair(u, dis), 3));

			}
		}

	}

	string src_lat, src_lon, dst_lat, dst_lon;
	//cin >> src_lat >> src_lon >> dst_lat >> dst_lon;
	//shortest_path(src_lat, src_lon, dst_lat, dst_lon);

	// test();
	 test_a_start();
	//cout << compressId << endl;
	//cout << "complete" << endl;
	return 0;
}
