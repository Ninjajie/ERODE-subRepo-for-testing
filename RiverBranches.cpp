// RiverBranches.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "RiverBranches.h"
#include "RiverNetwork.h"
#include <algorithm>
#include <fstream>

extern vector<pair<int, int>> branchIndices(RiverBranch* branch, double step);

int RiverBranch::index = 0;
int RiverNode::index = 0;

RiverNode::RiverNode()
	:priority(1), position(vec3(0, 0, 0)), parent(nullptr), terminal(false)
{
	index++;
	id = index;
}


RiverNode::RiverNode(int p, vec3 pos, RiverNode* parent)
	: priority(p), position(pos), parent(parent), terminal(false)
{
	index++;
	id = index;
}

RiverNode::~RiverNode()
{
	//todo?
}

void RiverNode::setElevation(double z)
{
	position[2] = z;
}

RiverBranch::RiverBranch(RiverNode * s, RiverNode * e)
	:start(s), end(e)
{
	index++;
	id = index;
}

double RiverBranch::distance(RiverBranch* branch) {

	//distance from this to branch
	double dis = DBL_MAX;

	RiverBranch* base = branch, *cmp = this;
	
	int num = 0;

	while (num < 2) {

		double dis1 = DBL_MAX;

		//make sure that x-axis of start point is less than that of end point
		vec3 start = branch->start->position, end = branch->end->position;
		if (start[0] > end[0]) swap(start, end);

		//calculate direction of branch
		double dx = end[0] - start[0];
		double dy = end[1] - start[1];
		vec2 dir = vec2(dx, dy);
		double t = dir.Length();
		dir.Normalize();
		vec2 per = vec2(-dy, dx);
		per.Normalize();

		//calculate endpoints of this w.r.t branch start point
		vec3 startT = this->start->position, endT = this->end->position;
		if (startT[0] > endT[0]) swap(startT, endT);

		vec2 offsetS = vec2(startT[0] - start[0],
			startT[1] - start[1]);
		vec2 offsetE = vec2(endT[0] - start[0],
			endT[1] - start[1]);

		//t1 and t2 are projection on dir 
		double t1 = Dot(offsetS, dir), t2 = Dot(offsetE, dir);
		if (t2 < 0.0) {
			dis1 = offsetS.Length();
		}
		else if (t1 > t) {
			vec2 d = vec2(startT[0] - end[0], startT[1] - end[1]);
			dis1 = d.Length();
		}
		else {
			cout << "enter per, num = " << num << endl;
			if (t1 < 0.0) {
				dis1 = abs(Dot(offsetE, per));
			}
			else {
				dis1 = abs(Dot(offsetS, per));
			}
		}

		dis = min(dis, dis1);
		num++;
		swap(base, cmp);
	}

	return dis;

}

double RiverNode::getElevation(double H, double W)
{
	return 100.0 * exp(-pow(position[0] - H / 2.0, 2.0) / 2000 - pow(position[1] - W / 2.0, 2.0) / 2000);
}

// main function
int main()
{
	RiverNetwork RN = RiverNetwork(100, 100, 10);
	ofstream outX("../TestImage/x.txt");
	ofstream outY("../TestImage/y.txt");
	ofstream outId("../TestImage/index.txt");
	ofstream outBranchX("../TestImage/branchx.txt");
	ofstream outBranchY("../TestImage/branchy.txt");
	ofstream outPri("../TestImage/pri.txt");
	RN.initialNode();

	//RiverNode* cNode = RN.selectNode(0.0);
	//RN.expandNode(cNode);
	for (int i = 0; i < 1000; i++)
	{
		RiverNode* cNode = RN.selectNode();
		if (cNode == nullptr)
		{
			break;
		}
		RN.expandNode(cNode);
	}
	cout << RN.nodes.size() << endl;
	for (int i = 0; i < RN.nodes.size(); i++)
	{

		std::cout << RN.nodes[i]->position << " " << RN.nodes[i]->id << std::endl;

		outX << RN.nodes[i]->position[0] << endl;
		outY << RN.nodes[i]->position[1] << endl;
		outId << RN.nodes[i]->id << endl;
		outPri << RN.nodes[i]->priority << endl;
	}
	for (int i = 0; i < RN.branches.size(); i++) {
		outBranchX << RN.branches[i]->start->position[0] << " " << RN.branches[i]->end->position[0] << endl;
		outBranchY << RN.branches[i]->start->position[1] << " " << RN.branches[i]->end->position[1] << endl;
	}

	//test branch distance
	RiverBranch* cur = RN.branches[min((unsigned int)10, RN.branches.size() - 1)];
	cout << "cur branch = " << cur->start->position << " / " << cur->end->position << endl;
	vector<pair<int, int>> idx = branchIndices(cur, 7.5);
	for (auto id : idx) {
		cout << id.first << " " << id.second << endl;
	}

	//RiverBranch* cmp = RN.branches[2];
	//
	//cout << "test branch distance function" << endl;
	//cout << "cur branch = " << cur->start->position << " / " << cur->end->position << endl;
	//cout << "cmp branch = " << cmp->start->position << " / " << cmp->end->position << endl;
	//cout << cur->distance(cmp) << endl;


	std::cout << "testout" << std::endl;
	system("pause");
    return 0;
}

