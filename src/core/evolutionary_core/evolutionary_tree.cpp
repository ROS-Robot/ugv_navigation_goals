#include "../../../include/header.hpp"

/* essentially, we want to create a tree from a given inorder traversal (the given path) */
Tree::Tree(std::vector<Waypoint> & path) {
    this->rootPtr = NULL;   // initially our rootPtr points nowhere
    /* for the first sub-tree we consider that we already have it's left child-node, it is our start position */
    int subtree_nodes = 1;
    std::stack<TreeNode*> helper;
    /* don't go until the goal node */
    for (std::vector<Waypoint>::iterator it = path.begin(); it != std::prev(path.end(), 1); it++) {
        TreeNode * nodePtr = new TreeNode(*it);

        if (helper.empty()) {
            helper.push(nodePtr);
        }
        else {
            if (helper.top()->leftPtr != NULL && helper.top()->rightPtr == NULL) {
                if (helper.top()->leftPtr->leftPtr != NULL) {
                    helper.push(nodePtr);
                }
                else {
                    helper.top()->rightPtr = nodePtr;
                }
            }
            else if (helper.top()->leftPtr != NULL && helper.top()->rightPtr != NULL) {
                assert(helper.size() == 1 || helper.size() == 2);
                if (helper.size() > 1) {
                    TreeNode * temp = helper.top();
                    helper.pop();
                    helper.top()->rightPtr = temp;
                }
                nodePtr->leftPtr = helper.top();
                helper.pop();
                helper.push(nodePtr);
            }
            else if (helper.top()->leftPtr == NULL && helper.top()->rightPtr == NULL) {
                nodePtr->leftPtr = helper.top();
                helper.pop();
                helper.push(nodePtr);
            }
            else {
                assert(1);
            }
        }

        subtree_nodes++;
    }
    /* we know which node is going to be the tree's root */
    this->rootPtr = helper.top();
    /* remove the start position, which is the leftmost node */
    TreeNode * temp = this->rootPtr;
    while (temp != NULL) {
        if (temp->leftPtr->leftPtr == NULL) {
            delete temp->leftPtr;
            temp->leftPtr = NULL;
        }
        temp = temp->leftPtr;
    }
    /* print the tree inorder -- for debugging */
    this->print();
}

/* print the tree inorder -- for debugging */
void Tree::print() {
    this->rootPtr->printInOrder();
}

/* help to print the tree inorder -- for debugging */
void TreeNode::printInOrder() {
    if (this->leftPtr != NULL)
        this->leftPtr->printInOrder();
    
    ROS_INFO("(x, y) = (%f, %f)", this->waypoint.pose.pose.position.x, this->waypoint.pose.pose.position.y);

    if (this->rightPtr != NULL)
        this->rightPtr->printInOrder();
}