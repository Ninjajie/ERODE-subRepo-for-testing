#pragma once

#include "RiverBranches.h"
#include "jc_voronoi.h"
#include <vector>

// class of cell
class Cell
{
public:

	RiverNode* center;
	vector<vec2> corners;

	Cell(RiverNode* center = nullptr);
	~Cell();
	float getArea();
};


// class of river network
class RiverNetwork
{
public:
	int width, height;

	double e;

	int numW, numH;

	vector<RiverNode*> nodes;
	vector<RiverBranch*> branches;
	vector<Cell*> cells;
	vector<vector<RiverBranch*>> grids;
	//this vector stores all the non-terminal nodes that currently can be expanded
	vector<RiverNode*> nonTerminalNodes;
	//vector<vector<double>> elevationMap;

	//current maximum priority
	int maxP;
	//current minimum elevation
	double minElevation;
	double elevationRange;
	//RiverNetwork();
	RiverNetwork(int w = 0, int h = 0, double e = 0.0);
	~RiverNetwork();

	//create the nodes to start from
	void initialNode();
	//select candidate nodes for expansion
	RiverNode* selectNode();
	//node expansion operation
	void expandNode(RiverNode* node);
	//get next candidate node
	RiverNode* getCandidate(RiverNode* parent, double angle, int p);
	//validation of the new node
	bool validateNode(RiverNode* node, double boundary, RiverBranch* branch);
    //this function is account for computing the minimum elevation of current non-terminal nodes
	void refreshMinele();

	//auxiliary function for branching operations
	int SymmetricBranching(RiverNode*);
	int AsymmetricBranching(RiverNode*);
	int Continuation(RiverNode*);

	//transform the points into jc_voronoi format
	jcv_point* transformFromNodes();
	jcv_rect* getRectFromNodes();
	
	//generate the tessellation
	jcv_diagram* voronoiTessellation();
	//transform the voronoi tessellation into cells
	void fillCells(jcv_diagram* diagram);
};