/* kd_tree.hpp

 * Copyright (C) 2021 SS47816
 
 * Implementation of the KD-Tree Data Structure

**/

#pragma once

#include <vector>

template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT setPoint, int setId) : point(setPoint), id(setId), left(NULL), right(NULL) {};
};

template<typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree() : root(NULL) {};

    void insertHelper(Node<PointT>** node, uint depth, const PointT& point, int id)
    {
        // Tree is empty
        if (*node == NULL)
            *node = new Node<PointT>(point, id);
        else
        {
            // Calulate current dimension
            uint cd = depth % 3;
            // if(point[cd] < ((*node)->point[cd]))
            if (point.data[cd] < ((*node)->point.data[cd]))
                insertHelper(&(*node)->left, depth + 1, point, id);
            else
                insertHelper(&(*node)->right, depth + 1, point, id);
        }
    }

    void insert(const PointT& point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(const PointT& target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if (node != NULL)
        {
            if (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)
				&& node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)
                && node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))
			{
				float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) 
									+ (node->point.y - target.y)*(node->point.y - target.y)
                                    + (node->point.z - target.z)*(node->point.z - target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

            // check accross boundary
            if ((target.data[depth%3] - distanceTol) < node->point.data[depth%3])
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            if ((target.data[depth%3] + distanceTol) > node->point.data[depth%3])
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }
    
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const PointT& target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};