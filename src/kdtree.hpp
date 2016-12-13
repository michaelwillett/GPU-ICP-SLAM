/**
* @file      kdtree.hpp
* @brief     intializer for 3-axis kd-tree
* @authors   Michael Willett
* @date      2016
* @copyright Michael Willett
*/

#pragma once

#include <glm/vec4.hpp>
#include <vector>

namespace KDTree
{
	class Node {
	public:
		Node();
		Node(glm::vec4  p, int state, int source);
		
		int axis;
		int left;
		int right;
		int parent;
		glm::vec4 value;

	};

	void Create(std::vector<glm::vec4> input, Node *list);
	void InsertList(std::vector<glm::vec4> input, Node *list, int idx, int parent);
	void InsertNode(glm::vec4 point, Node *list, int listSize);
};
