/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int                id;
    Node*              left;
    Node*              right;

    Node(std::vector<float> arr, int setId)
      : point(arr)
      , id(setId)
      , left(NULL)
      , right(NULL) {
    }

    ~Node() {
        delete left;
        delete right;
    }
};

struct KdTree {
    Node* root;

    KdTree()
      : root(NULL) {
    }

    ~KdTree() {
        delete root;
    }

    void insertHelper(Node** node, uint depth, std::vector<float> point, int id) {
        // Tree is empty
        if (*node == NULL) {
            *node = new Node(point, id);
        } else {
            // Calculate current dimension (cd)
            uint cd = depth % 2;  // 0 for x, 1 for y

            // Compare point with root on current dimension
            if (point[cd] < ((*node)->point[cd])) {
                insertHelper(&((*node)->left), depth + 1, point, id);
            } else {
                insertHelper(&((*node)->right), depth + 1, point, id);
            }
        }
    }

    void insert(std::vector<float> point, int id) {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(std::vector<float> target_point, Node* node, uint depth, float distanceTol, std::vector<int>& ids) {
        if (node != NULL) {
            float node_x = node->point[0];
            float node_y = node->point[1];

            float target_x = target_point[0];
            float target_y = target_point[1];

            if ((node_x >= (target_x - distanceTol) && node_x <= (target_x + distanceTol)) &&
                (node_y >= (target_y - distanceTol) && node_y <= (target_y + distanceTol))) {
                float distance = std::sqrt((target_x - node_x) * (target_x - node_x) + (target_y - node_y) * (target_y - node_y));

                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

            uint cd = depth % 2;  // current dimension. 0 for x, 1 for y
            if (target_point[cd] - distanceTol < node->point[cd]) {
                searchHelper(target_point, node->left, depth + 1, distanceTol, ids);
            }

            if (target_point[cd] + distanceTol > node->point[cd]) {
                searchHelper(target_point, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> ids;

        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};
