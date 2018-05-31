#include "../../../include/header.hpp"

/* essentially, we want to create a tree from a given inorder traversal (the given path) */
Tree::Tree(std::vector<Waypoint> & path) {
    ROS_INFO("Tree in");

    this->rootPtr = NULL;   // initially our rootPtr points nowhere
    /* for the first sub-tree we consider that we already have it's left child-node, it is our start position */
    std::stack<TreeNode*> helper;
    /* don't go until the goal node */
    ROS_INFO("Tree entering loop");
    for (std::vector<Waypoint>::iterator it = path.begin(); it != std::prev(path.end(), 1); it++) {
        TreeNode * nodePtr = new TreeNode(*it);

        if (helper.empty()) {
            helper.push(nodePtr);
        }
        else if (helper.top()->leftPtr == NULL && helper.top()->rightPtr == NULL) {
            nodePtr->leftPtr = helper.top();
            helper.pop();
            helper.push(nodePtr);
        }
        else if (helper.top()->rightPtr == NULL && 
                (helper.top()->leftPtr != NULL && helper.top()->leftPtr->leftPtr == NULL && helper.top()->leftPtr->rightPtr == NULL)) {
            helper.top()->rightPtr = nodePtr;
        }
        else if (helper.top()->rightPtr == NULL &&
                (helper.top()->leftPtr != NULL && helper.top()->leftPtr->leftPtr != NULL && helper.top()->leftPtr->rightPtr != NULL)) {
            helper.push(nodePtr);
        }

        if (helper.size() > 1) {
            /* for debugging */
            assert(helper.size() == 2);
            /* if we have a complete binary tree on top */
            if (helper.top()->leftPtr != NULL && helper.top()->rightPtr != NULL) {
                TreeNode * temp = helper.top();
                helper.pop();
                /* for debugging */
                assert(helper.top()->leftPtr != NULL && helper.top()->rightPtr == NULL);
                helper.top()->rightPtr = temp;
            }
        }
    }
    ROS_INFO("Tree exited loop");
    /* for debugging */
    assert(helper.size() == 1);
    /* we know which node is going to be the tree's root */
    this->rootPtr = helper.top();
    /* remove the start position, which is the leftmost node */
    TreeNode * temp = this->rootPtr;
    /* for debugging */
    assert(temp != NULL);
    while (temp->leftPtr != NULL) {
        if (temp->leftPtr->leftPtr == NULL) {
            delete temp->leftPtr;
            temp->leftPtr = NULL;
        }
        else
            temp = temp->leftPtr;
    }

    ROS_INFO("Tree out");
}

/* print the tree inorder -- for debugging */
void Tree::print() {
    ROS_INFO("print in");
    this->rootPtr->printInOrder();
    ROS_INFO("print out");
}

/* help to print the tree inorder -- for debugging */
void TreeNode::printInOrder() {
    ROS_INFO("printInOrder in");

    if (this->leftPtr != NULL)
        this->leftPtr->printInOrder();
    
    ROS_WARN("(x, y) = (%f, %f)", this->waypoint.pose.pose.position.x, this->waypoint.pose.pose.position.y);

    if (this->rightPtr != NULL)
        this->rightPtr->printInOrder();
    
    ROS_INFO("printInOrder out");
}