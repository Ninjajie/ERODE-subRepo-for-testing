// RiverBranches.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "RiverBranches.h"
#include "RiverNetwork.h"
int RiverBranch::id = 0;

RiverNode::RiverNode()
	:priority(1), position(vec3(0, 0, 0)), parent(nullptr), terminal(false)
{
}


RiverNode::RiverNode(int p, vec3 pos, RiverNode* parent)
	: priority(p), position(pos), parent(parent), terminal(false)
{
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
	id++;
}

double RiverNode::getElevation(double H, double W)
{
	return 100.0 * exp(-pow(position[0] - H / 2.0, 2.0) / 2000 - pow(position[1] - W / 2.0, 2.0) / 2000);
}

// main function
int main()
{
	RiverNetwork RN = RiverNetwork(100, 100, 10);
	RN.initialNode();
	//for (int i = 0; i < RN.elevationMap.size(); i++)
	//{
	//	for (int j = 0; j < RN.elevationMap[i].size(); j++)
	//	{
	//		std::cout << RN.elevationMap[i][j] << " ";
	//		if (j == RN.elevationMap[i].size() - 1)std::cout << std::endl;
	//	}
	//}
	for (int i = 0; i < 1000; i++)
	{
		std::cout << i << std::endl;
		RiverNode* cNode = RN.selectNode();
		if (cNode == nullptr)
		{
			break;
		}
		RN.expandNode(cNode);
	}
	for (int i = 0; i < RN.nodes.size(); i++)
	{
		std::cout << RN.nodes[i]->priority << std::endl;
		std::cout << RN.nodes[i]->position << std::endl;
	}
	std::cout << "testout" << std::endl;
	system("pause");
    return 0;
}

