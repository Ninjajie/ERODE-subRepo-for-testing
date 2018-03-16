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

RiverBranch::RiverBranch(RiverNode * s, RiverNode * e)
	:start(s), end(e)
{
	id++;
}

// main function
int main()
{
	RiverNetwork RN = RiverNetwork(100, 100, 10);
	std::cout << RN.numH << " " << RN.numW << endl;
	RN.initialNode();
	//RiverNode* cNode = RN.selectNode(0.0);
	//RN.expandNode(cNode);
	for (int i = 0; i < 1000; i++)
	{
		std::cout << i << std::endl;
		RiverNode* cNode = RN.selectNode(0.0);
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

