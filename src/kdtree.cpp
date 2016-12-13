/**
 * @file      kdtree.cpp
 * @brief     intializer for 3-axis kd-tree
 * @authors   Michael Willett
 * @date      2016
 * @copyright Michael Willett
 */

#include <algorithm>
#include "kdtree.hpp"

bool sortX(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.x < p2.x;
}
bool sortY(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.y < p2.y;
}
bool sortZ(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.z < p2.z;
}

void KDTree::Create(std::vector<glm::vec4> input, Node *list)
{
	std::sort(input.begin(), input.end(), sortX);
	InsertList(input, list, 0, -1);
}


void KDTree::InsertList(std::vector<glm::vec4> input, Node *list, int idx, int parent)
{
	int axis = (parent == -1) ? 0 : (list[parent].axis + 1) % 3;
	if (axis == 0)
		std::sort(input.begin(), input.end(), sortX);
	if (axis == 1)
		std::sort(input.begin(), input.end(), sortY);
	if (axis == 2 )
		std::sort(input.begin(), input.end(), sortZ);

	// set current node
	int mid = (int) input.size() / 2;
	list[idx] = Node(input[mid], axis, parent);

	if (mid > 0) {
		list[idx].left = idx + 1;
		std::vector<glm::vec4> left(input.begin(), input.begin() + mid);
		InsertList(left, list, list[idx].left, idx);
	}

	if (mid < input.size() - 1) {
		list[idx].right = idx + mid + 1;
		std::vector<glm::vec4> right(input.begin() + mid + 1, input.end());
		InsertList(right, list, list[idx].right, idx);
	}
}

void KDTree::InsertNode(glm::vec4 point, Node *list, int listSize) 
{
	int next = 0, parent = 0, axis = 0;
	do {	// traverse the list
		parent = next;
		axis = (list[next].parent == -1) ? 0 : (list[list[next].parent].axis + 1) % 3;
		if (axis == 0)
			next = (sortX(point, list[next].value)) ? list[next].left : list[next].right;
		if (axis == 1)
			next = (sortY(point, list[next].value)) ? list[next].left : list[next].right;
		if (axis == 2)
			next = (sortZ(point, list[next].value)) ? list[next].left : list[next].right;
	} while (next != -1);

	// update next pointer for end of list
	if (axis == 0) {
		if (sortX(point, list[parent].value))
			list[parent].left = listSize;
		else
			list[parent].right = listSize;
	}
	if (axis == 1) {
		if (sortY(point, list[parent].value))
			list[parent].left = listSize;
		else
			list[parent].right = listSize;
	}
	if (axis == 2) {
		if (sortZ(point, list[parent].value))
			list[parent].left = listSize;
		else
			list[parent].right = listSize;
	}

	// add new node to end of list
	list[listSize] = Node(point, (axis + 1) % 3, parent);
}

KDTree::Node::Node() {
	left = -1;
	right = -1;
	parent = -1;
	value = glm::vec4(0, 0, 0, 0);
	axis = 0;
}

KDTree::Node::Node(glm::vec4  p, int state, int source) {
	left = -1;
	right = -1;
	parent = source;
	value = p;
	axis = state;
}
