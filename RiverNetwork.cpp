#include "stdafx.h"
#include "RiverNetwork.h"
#include <random>
#include <algorithm>
#include <cmath>
#include <utility>
#include <fstream>

#include "bitmap_image.hpp"

#define EPSILON 0.0001

#define DisRatio 0.75

#define BranchLength 30

#define ElevationConstraint 12

#define NEEDW 46

#define PIXELX 1024

pair<int, int> mapGrid(vec3 pos, double step) {
	int i = std::floor(pos[0] / step);
	int j = std::floor(pos[1] / step);
	return pair<int, int>(i, j);
}


vector<pair<int, int>> branchIndices(RiverBranch* branch, double step, bool pixelwise) {

	vec3 startP = branch->start->position;
	vec3 endP = branch->end->position;

	vector<pair<int, int>> res;

	double deltax = endP[0] - startP[0];
	double deltay = endP[1] - startP[1];
	//if the line segment is near perpendicular
	if (abs(deltax) < EPSILON) {
		//swap(start, end);
		swap(deltax, deltay);
		swap(startP[0], startP[1]);
		swap(endP[0], endP[1]);
		pair<int, int> start = mapGrid(startP, step);
		pair<int, int> end = mapGrid(endP, step);
		if (start.first > end.first) {
			std::swap(startP, endP);
			std::swap(start, end);
		}
		int maxX = NEEDW;
		double singleStep = BranchLength * DisRatio;
		if (pixelwise)
		{
			maxX = PIXELX;
			singleStep = 1.0;
		}
		for (int x = start.first; x <= end.first && x < maxX; ++x) {
			double s0 = max(startP[0], x * singleStep);
			double s1 = min(endP[0], (x + 1) * singleStep);

			//cout << "s0 = " << s0 << ", s1 = " << s1 << endl;

			double y1 = (s0 - startP[0]) * deltay / deltax + startP[1];
			double y2 = (s1 - startP[0]) * deltay / deltax + startP[1];

			int ys = std::floor(y1 / step);
			int ye = std::floor(y2 / step);

			if (ys > ye) {
				swap(ys, ye);
			}
			for (int y = ys; y <= ye && y < maxX; ++y) {
				res.push_back(pair<int, int>(y, x));
				//cout << "x = " << x << ", y = " << y << endl;
			}
		}
		return res;
	}
	//else, as normal
	pair<int, int> start = mapGrid(startP, step);
	pair<int, int> end = mapGrid(endP, step);

	if (start.first > end.first) {
		std::swap(startP, endP);
		std::swap(start, end);
	}
	int maxX = NEEDW;
	double singleStep = BranchLength * DisRatio;
	if (pixelwise)
	{
		maxX = PIXELX;
		singleStep = 1.0;
	}
	for (int x = start.first; x <= end.first && x < maxX; ++x) {
		double s0 = max(startP[0], x * singleStep);
		double s1 = min(endP[0], (x + 1) * singleStep);


		//cout << "s0 = " << s0 << ", s1 = " << s1 << endl;

		double y1 = (s0 - startP[0]) * deltay / deltax + startP[1];
		double y2 = (s1 - startP[0]) * deltay / deltax + startP[1];

		int ys = std::floor(y1 / step);
		int ye = std::floor(y2 / step);

		if (ys > ye) {
			swap(ys, ye);
		}
		for (int y = ys; y <= ye; ++y) {
			res.push_back(pair<int, int>(x, y));
			//cout << "x = " << x << ", y = " << y << endl;
		}
	}
	return res;
}


RiverNetwork::RiverNetwork(int w, int h, double e)
	:width(w), height(h), e(e),minElevation(0.0)
{
	numW = std::ceil((double)width / (DisRatio * e));
	numH = std::ceil((double)height / (DisRatio * e));
	grids.resize(numH * numW, vector<RiverBranch*>());
	elevationRange = 25.0;
}

RiverNetwork::~RiverNetwork()
{
	//delete all the nodes
	for (int i = 0; i < nodes.size(); i++)
	{
		delete nodes[i];
	}
}

//after we import an image, we may need to resize the river map and therefore the underlying grids
void RiverNetwork::Resize(int newWidth, int newHeight)
{
	this->width = newWidth;
	this->height = newHeight;
	numW = std::ceil((double)width / (DisRatio * e));
	numH = std::ceil((double)height / (DisRatio * e));
	grids.resize(numH * numW, vector<RiverBranch*>());
}

//select the nodes to start from
//here is the simplified version, where we choose a random position on each edge
//the initial nodes all have priority of 1, which is the lowest priority
void RiverNetwork::initialNode()
{
	//create random real numbers between [0,1]
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);
	//multiply those random numbers by actual dimensions to get random positions on edges
	double ratio1 = dis(gen);
	double l1 = width * ratio1;
	double ratio2 = dis(gen);
	double l2 = height * ratio2;
	double ratio3 = dis(gen);
	double l3 = width * ratio3;
	double ratio4 = dis(gen);
	double l4 = height * ratio4;

	//randomly determine the starting priorities
	int p[4];
	for (int i = 0; i < 4; i++)
	{
		double neta = dis(gen);
		if (neta >= 0.7) {
			p[i] = 10;
		}
		else {
			p[i] = 11;
		}
	}
	//create 4 mouths around the boundary
	RiverNode* mouth1 = new RiverNode(p[0], vec3(l1, 0, 0), nullptr);
	nodes.push_back(mouth1);
	RiverNode* mouth2 = new RiverNode(p[1], vec3((double)width, l2, 0), nullptr);
	nodes.push_back(mouth2);
	RiverNode* mouth3 = new RiverNode(p[2], vec3(l3, (double)height, 0), nullptr);
	nodes.push_back(mouth3);
	RiverNode* mouth4 = new RiverNode(p[3], vec3(0, l4, 0), nullptr);
	nodes.push_back(mouth4);

	//we first apply a continuation for the initial mouths

	RiverNode* mouth11 = new RiverNode(p[0], vec3(l1, e, 0), mouth1);
	mouth11->id = mouth1->id;
	//mouth11->setElevation(elevationMap[(int)(l1/ (DisRatio * e))][(int)(e/ (DisRatio * e))]);
	mouth11->position[2] = mouth11->getElevation(height, width, this->elevationMap);
	nodes.push_back(mouth11);
	nonTerminalNodes.push_back(mouth11);
	RiverBranch* branch1 = new RiverBranch(mouth1, mouth11);
	branches.push_back(branch1);
	vector<pair<int, int>> idx = branchIndices(branch1, e * DisRatio,false);
	for (auto id : idx) {
		grids[id.first * numW + id.second].push_back(branch1);
	}

	RiverNode* mouth22 = new RiverNode(p[1], vec3((double)width - e, l2, 0), mouth2);
	mouth22->id = mouth2->id;
	//mouth22->setElevation(elevationMap[(int)(((double)width - e) / (DisRatio * e))][(int)(l2 / (DisRatio * e))]);
	mouth22->position[2] = mouth22->getElevation(height, width, this->elevationMap);
	nodes.push_back(mouth22);
	nonTerminalNodes.push_back(mouth22);
	RiverBranch* branch2 = new RiverBranch(mouth2, mouth22);
	branches.push_back(branch2);
	idx = branchIndices(branch2, e * DisRatio, false);
	for (auto id : idx) {
		grids[id.first * numW + id.second].push_back(branch2);
	}

	RiverNode* mouth33 = new RiverNode(p[2], vec3(l3, (double)height - e, 0), mouth3);
	mouth33->id = mouth3->id;
	//mouth33->setElevation(elevationMap[(int)(l3 / (DisRatio * e))][(int)(((double)height - e) / (DisRatio * e))]);
	mouth33->position[2] = mouth33->getElevation(height, width, this->elevationMap);
	nodes.push_back(mouth33);
	nonTerminalNodes.push_back(mouth33);
	RiverBranch* branch3 = new RiverBranch(mouth3, mouth33);
	branches.push_back(branch3);
	idx = branchIndices(branch3, e * DisRatio, false);
	for (auto id : idx) {
		grids[id.first * numW + id.second].push_back(branch3);
	}

	RiverNode* mouth44 = new RiverNode(p[3], vec3(e, l4, 0), mouth4);
	mouth44->id = mouth4->id;
	//mouth44->setElevation(elevationMap[(int)(e / (DisRatio * e))][(int)(l4 / (DisRatio * e))]);
	mouth44->position[2] = mouth44->getElevation(height, width, this->elevationMap);
	nodes.push_back(mouth44);
	nonTerminalNodes.push_back(mouth44);
	RiverBranch* branch4 = new RiverBranch(mouth4, mouth44);
	branches.push_back(branch4);
	idx = branchIndices(branch4, e * DisRatio, false);
	for (auto id : idx) {
		grids[id.first * numW + id.second].push_back(branch4);
	}


	//initialize the minimum elevation
	minElevation = mouth11->position[2];
	for (int i = 0; i < 2; i++)
	{
		if (nonTerminalNodes[i]->position[2] <= minElevation)minElevation = nonTerminalNodes[i]->position[2];
	}
}


void RiverNetwork::refreshMinele()
{
	double tempEle = 10000;
	for (int i = 0; i < nonTerminalNodes.size(); ++i)
	{
		if (nonTerminalNodes[i]->position[2] < tempEle)
			tempEle = nonTerminalNodes[i]->position[2];
	}
	minElevation = tempEle;
	//std::cout << "minEle" << minElevation << std::endl;
}
//from all the non-terminal nodes, select exactly one node that is subject to expansion
//based on the elevationRange and priorities
RiverNode * RiverNetwork::selectNode()
{
	
	//loop in all the non-terminal nodes, find the one that has the highest priority that lies within the 
	//elevation range of [z, z+elevationRange]

	if (nonTerminalNodes.size() == 0)
	{
		return nullptr;
	}
	
	//first need to re-compute the min elevation of current non-terminal nodes
	this->refreshMinele();
	//cadidateNodes is the set of all nodes with z,z+eR
	vector<RiverNode*> candidateNodes;
	//find the nodes within [z, z+elevationRange]
	for (int i = 0; i < nonTerminalNodes.size(); i++)
	{
		if (nonTerminalNodes[i]->position[2] <= minElevation + elevationRange)
			candidateNodes.push_back(nonTerminalNodes[i]);
	}
	//find the highest priority value in candidateNodes
	//if (candidateNodes.size() == 0) {
	//	elevationRange
	//}
	int maxP = candidateNodes[0]->priority;
	for (int i = 0; i < candidateNodes.size(); i++)
	{
		if (candidateNodes[i]->priority >= maxP) maxP = candidateNodes[i]->priority;
	}
	vector<RiverNode*> finalcandidateNodes;
	// if all the nonterminal is of priority 1, we increase all their priorities by 1
	if (maxP == 1)
	{
		maxP = 2;
		for (int i = 0; i < candidateNodes.size(); i++)
		{
			candidateNodes[i]->priority = maxP;
			finalcandidateNodes.push_back(candidateNodes[i]);
		}
	}
	else
	{
		//find the set of the nodes with the highest priority value
		for (int i = 0; i < candidateNodes.size(); i++)
		{
			if (candidateNodes[i]->priority == maxP)
				finalcandidateNodes.push_back(candidateNodes[i]);
		}
	}
	//if this set has more than one element, select the one with lowest elevation
	if (finalcandidateNodes.size() > 1)
	{
		
		int finalIndex = 0;
		double minele = finalcandidateNodes[0]->position[2];
		for (int i = 0; i < finalcandidateNodes.size(); i++)
		{
			if (finalcandidateNodes[i]->position[2] <= minele) {
				finalIndex = i;
				minele = finalcandidateNodes[i]->position[2];
			}
		}
		return finalcandidateNodes[finalIndex];
	}
	else {
		return finalcandidateNodes[0];
	}

}

//given the selected candidate node, choose from three different situations
//(symmetric/asymmetric/continuation)
//and compute new branches from them
void RiverNetwork::expandNode(RiverNode * node)
{
	//if priority >1 we have three different situations
	if (node->priority > 1)
	{
		double prob = (double)std::rand() / (double)RAND_MAX;
		// symmetric
		if (prob >= 0.0 && prob < 0.3) {
			this->SymmetricBranching(node);
		}
		// asymmetric
		else if (prob > 0.3 && prob <= 0.9) {
			this->AsymmetricBranching(node);
		}
		// continuation
		else {
			this->Continuation(node);
		}
	}
	//else only continuation
	else {
		this->Continuation(node);
	}
	// After expansion part, we need to set this node to terminal(remove it from nonTerminals
	int ind = 0;
	for (int i = 0; i < nonTerminalNodes.size(); i++)
	{
		if (nonTerminalNodes[i] == node) {
			ind = i;
			break;
		}
	}
	nonTerminalNodes.erase(nonTerminalNodes.begin() + ind);
	std::cout << nonTerminalNodes.size() << std::endl;

}

RiverNode* RiverNetwork::getCandidate(RiverNode* node, double angle, int p) {
	vec2 ppos = vec2(node->parent->position[0], node->parent->position[1]);
	vec2 pos = vec2(node->position[0], node->position[1]);
	vec2 dir = pos - ppos;
	dir = dir.Normalize();
	vec2 perpen = vec2(dir[1], -dir[0]);
	vec2 FinalDir = dir * std::sin(Deg2Rad * angle) + perpen * std::cos(Deg2Rad * angle);
	//double dx = e * std::cos(Deg2Rad * angle), dy = e * std::sin(Deg2Rad * angle);
	RiverNode* resultNode =  new RiverNode(p, vec3(pos[0] + e * FinalDir[0], pos[1] + e * FinalDir[1], node->position[2] /*+ elevation*/), node);
	resultNode->setElevation(resultNode->getElevation(height, width, this->elevationMap));
	return resultNode;
}


//In this function, the input is a possible new node to be added to the network
//this function check if this node is far enough from:
//1.the boundary of terrain and 
//2.all other branches


bool RiverNetwork::validateNode(RiverNode * node, double boundary, RiverBranch* branch)
{

	// check elevation correctness
	if (node->position[2] < node->parent->position[2] - ElevationConstraint) {
		return false;
	}
	// check if within boundary
	if (node->position[0] < boundary ||
		node->position[0] > (double)width - boundary ||
		node->position[1] < boundary ||
		node->position[1] > (double)height - boundary) {
		return false;
	}
	// check collision with other branches
	branch = new RiverBranch(node->parent, node);
	vector<pair<int, int>> indices = branchIndices(branch, e * DisRatio, false);
	pair<int, int> parentIdx = mapGrid(node->parent->position, e * DisRatio);
	for (auto id : indices) {
		//skip the parent grid
		if (id.first == parentIdx.first && id.second == parentIdx.second) continue;
		//for all the left indices, check if there are any branches inside
		int idx = id.first * numW + id.second;
		if (id.first >= 0 && id.first < numW &&
			id.second >= 0 && id.second < numH) 
		{
			//if there is a branch inside, then just return false
			if (grids[idx].size() != 0)
			{
				branch = nullptr;
				return false;
			}
			//else we need to check all the surronding grids and compute the distance
			else
			{
				for (int verInd = -1; verInd <= 1; verInd++)
				{
					for (int horInd = -1; horInd <= 1; horInd++)
					{
						int subidx = (id.first + horInd) * numW + (id.second + verInd);
						if (id.first + horInd >= 0 && id.first + horInd < numW &&
							id.second + verInd >= 0 && id.second + verInd< numH)
						{
							//if there is at least one branch inside, then compute the distance
							if (grids[subidx].size() != 0)
							{
								//iterate the branches inside, compute the distatnce
								for (int indBranches = 0; indBranches < grids[subidx].size(); indBranches++)
								{
									RiverBranch* existBranch = grids[subidx][indBranches];
									double distance = branch->distance(existBranch);
									if (distance >= DisRatio * BranchLength)
									{
										branch = nullptr;
										return false;
									}
								}
							}
						}
						//else out of range
						else continue;
					}
				}
			}
		}
		//our of range
		else {
			branch = nullptr;
			return false;
		}
	}
	//no problem, add this branch to branch list
	branches.push_back(branch);
	//tell grid that this branch is inside
	for (auto id : indices) {
		grids[id.first * numW + id.second].push_back(branch);
	}

	return true;

}

//auxiliary functions for branching
int RiverNetwork::SymmetricBranching(RiverNode* node)
{
	//check the final # of added nodes
	int nodesAdded = 0;
	//loop count
	int num = 2;
	while (num) {
		double initialAngle = 45.0 + 90.0 * (num % 2), currentAngle = initialAngle;
		double angleStep = 2.5;
		RiverNode* newNode = nullptr;
		RiverBranch* branch = nullptr;
		//currentAngle >= -2.5 && <= 182.5 means that it can surpass the bottom line of 0/180 degree for once.
		for (int k = 0; currentAngle >= 0 - angleStep && currentAngle <= 180 + angleStep; k++)
		{
			currentAngle = currentAngle + pow(-1.0, k) * (k / 2) * angleStep;
			int newP = node->priority - 1;
			if (newP < 1)newP = 1;
			newNode = getCandidate(node, currentAngle, newP);
			//if a new node is avaliable at some position, add this node to the node list 
			//also add this node to its parent's children list
			if (validateNode(newNode, e * DisRatio, branch))
			{
				newNode->id = node->id;
				node->children.push_back(newNode);
				this->nodes.push_back(newNode);
				//add this newNode to nonterminal 
				this->nonTerminalNodes.push_back(newNode);
				break;
			}
		}
		num--;
	}
	return nodesAdded;
}
int RiverNetwork::AsymmetricBranching(RiverNode* node)
{
    //check the final # of added nodes
	int nodesAdded = 0;
	//loop count
	int num = 2;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.7, 0.99);
	double ratio = dis(gen);
	int newP = (int)(ratio * node->priority);
	if (newP < 1)newP = 1;
	//set a random priority for the second node
	int p[2] = { node->priority, newP };
	while (num) {
		//int k = 0;
		double initialAngle = 45.0 + 90.0 * (num % 2), currentAngle = initialAngle;
		double angleStep = 2.5;
		RiverNode* newNode = nullptr;
		RiverBranch* branch = nullptr;
		//currentAngle >= -2.5 && <= 182.5 means that it can surpass the bottom line of 0/180 degree for once.
		for (int k = 0; currentAngle >= 0 - angleStep && currentAngle <= 180 + angleStep; k++)
		{
			currentAngle = currentAngle + pow(-1.0, k) * (k / 2) * angleStep;
			newNode = getCandidate(node, currentAngle, p[num - 1]);
			//if a new node is avaliable at some position, add this node to the node list 
			//also add this node to its parent's children list
			//also add the branch to the branch list
			if (validateNode(newNode, e * 0.25, branch))
			{
				newNode->id = node->id;
				node->children.push_back(newNode);
				this->nodes.push_back(newNode);
				//add this to nonterminal 
			    this->nonTerminalNodes.push_back(newNode);
				nodesAdded++;
				break;
			}
		}
		num--;
	}
	return nodesAdded;
}
int RiverNetwork::Continuation(RiverNode* node)
{
	int k = 0;
	double initialAngle = 90.0, currentAngle = initialAngle;
	double angleStep = 2.5;
	RiverNode* newNode = nullptr;
	RiverBranch* branch = nullptr;
	//currentAngle >= -2.5 && <= 182.5 means that it can surpass the bottom line of 0/180 degree for once.
	for (int k = 0; currentAngle >= 0 - angleStep && currentAngle <= 180 + angleStep; k++)
	{
		currentAngle = currentAngle + pow(-1, k) * (k / 2) * angleStep;
		newNode = getCandidate(node, currentAngle, node->priority);

		//if a new node is avaliable at some position, add this node to the node list 
		//also add this node to its parent's children list
		if (validateNode(newNode, e * 0.25, branch))
		{
			newNode->id = node->id;
			node->children.push_back(newNode);
			this->nodes.push_back(newNode);
			//add this to nonterminal 
			this->nonTerminalNodes.push_back(newNode);
			return 1;
		}
	}
	return 0;
}

//constuct the structure for voronoi tessellation
jcv_point * RiverNetwork::transformFromNodes()
{
	int networkSize = this->nodes.size();
	jcv_point* points = new jcv_point[networkSize];
	for (int i = 0; i < networkSize; i++)
	{
		jcv_point jp;
		jp.x = this->nodes[i]->position[0];
		jp.y = this->nodes[i]->position[1];
		points[i] = jp;
	}
	return points;
}

//construct the rectangle for voronoi boundary
jcv_rect * RiverNetwork::getRectFromNodes()
{
	jcv_rect* rect = new jcv_rect();
	jcv_point* min = new jcv_point();
	min->x = 0.0;
	min->y = 0.0;
	jcv_point* max = new jcv_point();
	max->x = (double)this->width;
	max->y = (double)this->height;
	rect->min = *min;
	rect->max = *max;
	return rect;
}

jcv_diagram * RiverNetwork::voronoiTessellation()
{
	int numpoints = this->nodes.size();
	const jcv_point* points = this->transformFromNodes();
	const jcv_rect* boundary = this->getRectFromNodes();
	jcv_diagram* diagram = new jcv_diagram();
	memset(diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(numpoints, points, boundary, diagram);
	//remember to free the space after use!!!!!!
	return diagram;
}

void RiverNetwork::fillCells(jcv_diagram * diagram)
{
	const jcv_site* sites = jcv_diagram_get_sites(diagram);
	int numofCells = diagram->numsites;
	for (int i = 0; i < numofCells; ++i)
	{
		const jcv_site* site = &sites[i];
		//find the corresponding riverNode and assign it to a new cell
		int index = site->index;
		RiverNode* center = this->nodes[index];
		Cell* newcell = new Cell(center);
		//compute the corners and push into the cell
		const jcv_graphedge* e = site->edges;
		while (e)
		{
			vec2 corner = vec2(e->pos[0].x, e->pos[0].y);
			newcell->corners.push_back(corner);
			e = e->next;
		}
		this->cells.push_back(newcell);
	}
}

void RiverNetwork::readBMP(const std::string file)
{

	bitmap_image image(file);

	if (!image)
	{
		printf("Error - Failed to open: input.bmp\n");
		return;
	}

	unsigned int total_number_of_pixels = 0;

	const unsigned int height = image.height();
	const unsigned int width = image.width();



	ofstream heightValues("heightvalues.txt");
	// get the vector <R,G,B> for the pixel at (w,h)
	for (std::size_t y = 0; y < height; ++y)
	{
		std::vector<double> oneRow;
		for (std::size_t x = 0; x < width; ++x)
		{
			// get the vector <R,G,B> for the pixel at (1,1)
			rgb_t colour;
			image.get_pixel(x, y, colour);
			heightValues << static_cast<int>(colour.red) << std::endl;
			oneRow.push_back(static_cast<double>(colour.red));
		}
		this->elevationMap.push_back(oneRow);
	}

	printf("Number of pixels with red >= 111: %d\n", total_number_of_pixels);

}

void RiverNetwork::readElevation(const std::string elevationValues)
{
	ifstream elevations(elevationValues, std::ios::in);
	for (int i = 0; i < 1024; i++)
	{
		std::vector<double> oneRow;
		for (int j = 0; j < 1024; j++)
		{
			double toBeInput = 0.0;
			elevations >> toBeInput;
			oneRow.push_back(toBeInput);
		}
		this->elevationMap.push_back(oneRow);
	}
}

void RiverNetwork::writeRivers(const std::string filename)
{
	bitmap_image originHeightmap(filename);
	int w = originHeightmap.width();
	int h = originHeightmap.height();
	bitmap_image alteredHeightmap(w, h);
	//first we just copy the original image
	for (unsigned int y = 0; y < h; ++y)
	{
		for (unsigned int x = 0; x < w; ++x)
		{
			rgb_t colour;
			originHeightmap.get_pixel(x, y, colour);
			alteredHeightmap.set_pixel(x, y, colour);
		}
	}
	//then for all the branches, we alter the corresponding pixel values
	for (auto branch : this->branches)
	{
		//here for the only time we pass true to the pixelwise argument
		std::vector<pair<int, int>> corespondPixels = branchIndices(branch, 1.0, true);
		for (auto indices : corespondPixels)
		{
			rgb_t newColor;
			newColor.red = newColor.green = newColor.blue = static_cast<unsigned char>(0);
			alteredHeightmap.set_pixel(indices.first, indices.second, newColor);
		}
	}

	alteredHeightmap.save_image("alteredHeightmap.bmp");
}


//Voronoi cell class

//Cell constructor
Cell::Cell(RiverNode* center)
	:center(center)
{
	//Todo
}

//Cell destructor
Cell::~Cell()
{
	//Todo
}

//get the area of current cell
float Cell::getArea()
{
	//We use Shoelace formula to compute the area of a polygon
	double area = 0.0;
	int numofCorners = this->corners.size();
	int j = numofCorners - 1;
	for (int i = 0; i < numofCorners; i++)
	{
		area += (this->corners[j][0] + this->corners[i][0]) * (this->corners[j][1] - this->corners[i][1]);
		j = i;
	}
	return abs(area/2.0);
}
