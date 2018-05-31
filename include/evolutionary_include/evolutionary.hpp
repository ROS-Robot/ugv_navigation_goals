#pragma once

#include <stack>
#include <time.h>
#include <stdlib.h>

/* useful definitions */

#define GENERATION_SIZE 4
#define NUM_OF_BEST_FIT 2
#define MAX_GENERATIONS 10  // maximum number of generations of individuals to examine

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