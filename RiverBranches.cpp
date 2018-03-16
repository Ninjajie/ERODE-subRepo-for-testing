// RiverBranches.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "RiverBranches.h"
#include "RiverNetwork.h"
#include <fstream>

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

RiverBranch::RiverBranch(RiverNode * s, RiverNode * e)
	:start(s), end(e)
{
	index++;
	id = index;
}

// main function
int main()
{
	RiverNetwork RN = RiverNetwork(100, 100, 10);
	ofstream outX("x.txt");
	ofstream outY("y.txt");
	ofstream outId("index.txt");
	RN.initialNode();
	for (int i = 0; i < 100; i++)
	{
		RiverNode* cNode = RN.selectNode(0.0);
		if (cNode == nullptr)
		{
			break;
		}
		RN.expandNode(cNode);
	}
	for (int i = 0; i < RN.nodes.size(); i++)
	{
		std::cout << RN.nodes[i]->position << " " << RN.nodes[i]->id << std::endl;

		outX << RN.nodes[i]->position[0] << endl;
		outY << RN.nodes[i]->position[1] << endl;
		outId << RN.nodes[i]->id << endl;
	}
	for (int i = 0; i < RN.branches.size(); i++) {
		cout << RN.branches[i]->id << endl;
	}
	std::cout << "testout" << std::endl;
	system("pause");
    return 0;
}

