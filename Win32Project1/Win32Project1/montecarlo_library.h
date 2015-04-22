#define _USE_MATH_DEFINES
#include <utility>
#include <map>
#include <ctime>
#include <string>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <queue>          // std::priority_queue
#include <vector>
#include <list>
#include "stdafx.h"
#include "Win32Project1.h"
#include <set>

using namespace std;

class simulation;
class roomba_action;
const int numroombas = 10;


//naive handling of trials, just for testing
int num_sims = 1000;//we'll find a better way to manage the simulations later
double sim_duration = 20; //in seconds

// vals from arduino
double robotSpeed = (double)330/(double)1000; // mm/s
int randomMax = 64;
// Time between trajectory noise injections
double noiseInterval = (double)5000/(double)1000;
double lastNoise = 0;
// Time between auto-reverse
double reverseInterval = (double)20000/(double)1000;
double lastReverse = 0;
// Time needed to affect trajectory
double noiseLength = (double)850/(double)1000;
double beginNoise = 0;
// Time needed to reverse trajectory
double reverseLength = (double)2456/(double)1000; // .33/2 m/s * pi * wheelbase / 2
double beginReverse = 0;
// Time needed to spin 45 degrees
double topTouchTime = reverseLength / 4;
double beginTopTouch = 0;

double wheelbase_width = 0.25798379655423864; //worked backwards to this

double fix_angle(double in_afngle); //all in milliseconds
double roombaDiameter = 0.34;

double runendtime = 120; //default state end time

enum notification { TOUCHED, REVERSAL, NOISE, MAGNET, ENDED,TOUCHENDED};
enum state_type { REVERSE, RUN, RANDROT, TOPTOUCH, STOPPED,OBSTACLERUN,OBSTACLESTOPPED};

struct trial;
class roomba_action;
struct pqueue_index;
class simulation;

double fix_angle(double in_angle) {
	while (in_angle >= 2 * M_PI) {
		in_angle -= 2 * M_PI;
	}
	while (in_angle < 0) {
		in_angle += 2 * M_PI;
	}
	return in_angle;
}

struct pqueue_index {
	roomba_action * caller; //who we'll notify
	roomba_action * linked; //the possible roomba that 
	notification type;
	double at_time;
	pqueue_index(roomba_action * call, roomba_action * link, notification ty, double t) : caller(call), linked(link), type(ty), at_time(t) {}
	/*bool operator<(const pqueue_index & a) {
		return at_time < a.at_time;
	}*/
};

bool operator<(const pqueue_index&a, const pqueue_index & b) {
	return a.at_time > b.at_time;
}


struct trial {
	priority_queue<pqueue_index> organizer;
	trial(simulation * sims);
	simulation * c_sim;
	roomba_action * startbots[14];
	roomba_action * currbots[14];
	bool clist[14][14];
	void collision_handle();
	double currtime = 0;
	void evaluatetime(double until, HDC hdc);
};

class roomba_action { //I'm making it so velocity and rotation should be mutually exclusive(ish?)
public:
	void notify(double at_time, pqueue_index type);
	roomba_action * parent = nullptr;
	roomba_action * child = nullptr;

	state_type mytype;

	double next_noise;
	double next_reversal;
	bool is_obstacle = false;

	double start_time; //consider replacing vectors with lists
	double end_time; //have figured out they must have a known end time based on how they behave
	trial * c_test;
	simulation * c_sim;

	double rw_v = 0;
	double lw_v = 0;

	pair<double, double> startpos;
	double initial_angle;
	int ID;

	void addevent(double at_time, notification type);
	roomba_action(double at_time, roomba_action * par, state_type type);
	roomba_action(double s_time, pair<double, double> spos, double init_an, trial * curr_test, int i,bool killbot);
	~roomba_action();

	void spawn_state(double at_time, state_type st);
	pair<double, double> getposition(double at_time);
	double getrotation(double at_time);
	void initNotify();

};

//pointer-dependent

class simulation { //super master class thing monster creature
public:
	list<trial> tests;
	int testn = 0;
	double roomba_offset;
	double whack_offset;
	//vector<roomba*> robots;
	simulation(double offset);
};

simulation::simulation(double offset) {
	roomba_offset = offset;
	whack_offset = 0;
/*	for (int test_iter = 0; test_iter < num_sims; test_iter++) {
	
	}*/

	//tests.push_back(trial(this));
}

void trial::evaluatetime(double until,HDC hdc) {
	//if (!organizer.empty()) {	//int a = 0;
	while (currtime < until) {
		currtime += ((double)15/(double)1000);
		collision_handle();
		while (!organizer.empty()) {
			pqueue_index first = organizer.top();
			if (first.at_time <= currtime) {
				organizer.pop();
				//cout << first.caller->ID << endl;
				first.caller->notify(first.at_time, first); //send notification
			}
			else {
				break;
			}
		}
		for (int i = 0; i < 14; i++) {
			pair<double,double> st = currbots[i]->getposition(currtime-(double)15/(double)1000);
			pair<double, double> ed = currbots[i]->getposition(currtime);
			//cout << ed.first << " " << ed.second << endl;
			SelectObject(hdc, GetStockObject(DC_PEN));
			int color = (double)255 * ((until - currtime) / until);
			SetDCPenColor(hdc, RGB(255/3+color*2/3, 255/3+color*2/3, 255-color));
			MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
			LineTo(hdc, (int)40*ed.first, (int)40*ed.second);
		}
	}
}

double simp_dform(pair<double, double> a, pair<double, double> b) {
	return pow((a.first - b.first), 2) + pow((a.second - b.second), 2);
}

void trial::collision_handle() {
	for (int i = 0; i < 13; i++) {
		for (int j = i + 1; j < 14; j++) {
			pair<double, double> a = currbots[i]->getposition(currtime);
			pair<double, double> b = currbots[j]->getposition(currtime);
			if ((pow(roombaDiameter,2))>=simp_dform(a,b)) {
				if (fix_angle(atan2(b.second - a.second, b.first - a.first) - currbots[i]->getrotation(currtime) + M_PI / 2) <= M_PI) { //if other is in front
					if (!clist[i][j]) {
						clist[i][j] = true;
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHED, currtime));
					}
				}
				else {
					if (clist[i][j]) {
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime));
					}
					clist[i][j] = false;
				}
				if (fix_angle(atan2(a.second - b.second, a.first - b.first) - currbots[j]->getrotation(currtime) + M_PI / 2) <= M_PI) { //if other is in front
					if (!clist[j][i]) {
						clist[j][i] = true;
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHED, currtime));
					}
				}
				else {
					if (clist[j][i]) {
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime));
					}
					clist[j][i] = false;
				}
			}
			else {
				if (clist[i][j]) {
					organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime));
				}
				if (clist[j][i]) {
					organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime));
				}
				clist[i][j] = false;
				clist[j][i] = false;
			}
		}
	}
}

void roomba_action::addevent(double at_time, notification type) {
	c_test->organizer.push(pqueue_index(this, nullptr, type, at_time));
}


void roomba_action::initNotify() {
	switch (mytype){
	case RUN:
		rw_v = robotSpeed;
		lw_v = robotSpeed;
		if ((next_reversal <= next_noise) || (next_reversal <= start_time)) { //reversal takes precedent
			end_time = max(next_reversal, start_time);
			addevent(end_time, REVERSAL);
		}
		else {
			end_time = max(next_noise, start_time);
			addevent(end_time, NOISE);
		}
		break;
	case REVERSE:
		rw_v = -robotSpeed/2;
		lw_v = robotSpeed/2;
		end_time = start_time+reverseLength;
		addevent(end_time,ENDED);
		break;
	case RANDROT: {
		double randv = rand() % randomMax;
		randv = randv - randomMax / 2;
		randv /= 1000;
		rw_v = robotSpeed - randv;
		lw_v = robotSpeed + randv;
		end_time = start_time+noiseLength;
		addevent(end_time,ENDED);
		break;
	}
	case TOPTOUCH:
		rw_v = -robotSpeed / 2;
		lw_v = robotSpeed / 2;
		end_time = start_time+reverseLength/4;
		break;
	case OBSTACLERUN:
		rw_v = robotSpeed - (double)9/(double)1000;
		lw_v = robotSpeed + (double)9/(double)1000;
		end_time = runendtime;
		break;
	case OBSTACLESTOPPED:
		rw_v = 0;
		lw_v = 0;
		end_time = runendtime;
		break;
	default:
		break;
	}
}

double roomba_action::getrotation(double at_time) {
	double dt = at_time - start_time;
	if (fabs(rw_v - lw_v) < 1.0e-6) {
		return initial_angle;
	}
	else {
		double wd = (rw_v * dt - lw_v * dt) / wheelbase_width;
		return initial_angle + wd;
	}
}

pair<double, double> roomba_action::getposition(double at_time) {
	double dt = at_time - start_time;
	if (fabs(rw_v - lw_v) < 1.0e-6) {
		return make_pair(startpos.first + rw_v * dt * cos(initial_angle), startpos.second + rw_v * dt * sin(initial_angle));
	}
	else {
		double R = wheelbase_width * (rw_v * dt + lw_v * dt) / (2 * (rw_v * dt - lw_v * dt)),
		wd = (rw_v * dt - lw_v * dt) / wheelbase_width;
		return make_pair(startpos.first + R * sin(wd + initial_angle) - R * sin(initial_angle), startpos.second - R * cos(wd + initial_angle) + R * cos(initial_angle));
	}
}

roomba_action::~roomba_action() {
	//roomba_action * temp = this;
	if (parent != nullptr) {
		parent->child = nullptr;
	}
	if (child != nullptr) {
		child->parent = nullptr;
		delete child;
	}
}

roomba_action::roomba_action(double s_time, pair<double, double> spos, double init_an,trial * curr_test,int i,bool killbot) { //init constructor
	if (!killbot) {
		mytype = RUN;
		next_noise = noiseInterval;
		next_reversal = reverseInterval;
	}
	else {
		mytype = OBSTACLERUN;
		next_noise = runendtime;
		next_reversal = runendtime;
	}
	is_obstacle = killbot;
	//c_robot = curr_robot;
	c_test = curr_test;
	c_sim = curr_test->c_sim;
	parent = nullptr;
	child = nullptr;
	startpos = spos;
	initial_angle = fix_angle(init_an);
	start_time = s_time;
	//cout << i;
	ID = i;
	initNotify();
}

roomba_action::roomba_action(double at_time,roomba_action * par,state_type type) { //standard constructor
	mytype = type;
	c_test = par->c_test;
	c_sim = c_test->c_sim;
	parent = par;
	child = nullptr;
	next_noise = par->next_noise;
	next_reversal = par->next_reversal;
	startpos = par->getposition(at_time);
	initial_angle = fix_angle(par->getrotation(at_time));
	start_time = at_time;
	ID = par->ID;
	initNotify();
}

void roomba_action::spawn_state(double at_time,state_type st) {
	roomba_action * temp;
	temp = new roomba_action(at_time, this, st);
	end_time = at_time;
	child = temp;
	child->parent = this;
	c_test->currbots[ID] = temp;
}

trial::trial(simulation * sims) {
	c_sim = sims;
	for (int i = 0; i < 10; i++) {
		double ang = fix_angle((2 * M_PI / 10)*i/* + c_sim->roomba_offset*/);
		startbots[i] = new roomba_action(0, make_pair(10 + cos(ang), 10 + sin(ang)), ang, this, i,false);
		currbots[i] = startbots[i];
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
	for (int i = 0; i < 4; i++) {
		double ang = fix_angle((2 * M_PI / 4)*i/* + c_sim->whack_offset*/);
		startbots[10+i] = new roomba_action(0, make_pair(10 + 5*cos(ang), 10 + 5*sin(ang)), fix_angle(ang-M_PI/2), this, i+10,true);
		currbots[10+i] = startbots[i+10];
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
}

void roomba_action::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case TOUCHED:
			if ((mytype == RUN || mytype == NOISE || mytype == TOPTOUCH) && info.linked != nullptr) {
				if (info.linked->end_time >= at_time) {
					spawn_state(at_time,REVERSE);
				}
			}
			if (mytype == OBSTACLERUN && info.linked != nullptr) {
				if (info.linked->end_time >= at_time) {
					spawn_state(at_time, OBSTACLESTOPPED);
				}
			}
			break;
		case REVERSAL:
			if (mytype == RUN) {
				next_reversal = at_time + reverseInterval;
				spawn_state(at_time, REVERSE);
			}
			break;
		case NOISE:
			if (mytype == RUN) {
				next_noise = at_time + noiseInterval;
				spawn_state(at_time, RANDROT);
			}
			break;
		case MAGNET:
			if (mytype == RANDROT || mytype == REVERSE || mytype == RUN) {
				spawn_state(at_time, TOPTOUCH);
			}
			break;
		case ENDED:
			if (mytype == RANDROT || mytype == REVERSE || mytype == TOPTOUCH) {
				spawn_state(at_time, RUN);
			}
			break;
		case TOUCHENDED:
			if (mytype == OBSTACLESTOPPED) {
				spawn_state(at_time, OBSTACLERUN);
			}
			break;
		default:
			break;
		}
	}
}