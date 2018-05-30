#pragma once

#include <stack>

/* useful definitions */

#define NUM_OF_BEST_FIT 2

/* Our path's (binary) tree node structure */
class TreeNode {
public:
    Waypoint waypoint;
    TreeNode* leftPtr;
    TreeNode* rightPtr;

    /* constructor */
    TreeNode(Waypoint & w) 
        : waypoint(w), leftPtr(NULL), rightPtr(NULL) 
        {};
    /* destructor */
    ~TreeNode() {
        delete leftPtr;
        delete rightPtr;
    }
    /* help print the tree inorder -- for debugging */
    void printInOrder();
};

/* Our path's (binary) tree structure */
class Tree {
public:
    TreeNode* rootPtr;

    /* constructors */
    Tree() 
        : rootPtr(NULL) 
        {};
    /* create a path's binary tree */
    Tree(std::vector<Waypoint> & path);
    /* destructor */
    ~Tree() {
        delete rootPtr;
    }
    /* print the tree inorder -- for debugging */
    void print();
};