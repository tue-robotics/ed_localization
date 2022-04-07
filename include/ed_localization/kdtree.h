#ifndef ED_LOCALIZATION_KDTREE_H_
#define ED_LOCALIZATION_KDTREE_H_

#include <array>
#include <memory>
#include <vector>

#include <geolib/math_types.h>

namespace ed_localization {

struct KDTreeNode
{
    KDTreeNode() : depth(0), leaf(true), pivot_dim(0), pivot_value(0), key({0,0,0}), value(0), cluster(-1), children({nullptr, nullptr})
    {
    }

    // Depth in the tree
    unsigned int depth;

    // Leaf node
    bool leaf;

    // Pivot dimension and value
    unsigned int pivot_dim;
    double pivot_value;

    // The key for this node
    std::array<int, 3> key;

    // The value for this node, summed weights in this bin
    double value;

    // The cluster label (leaf nodes)
    int cluster;

    // Child nodes
   std::array<KDTreeNode*, 2> children;
};

class KDTree
{

public:

    KDTree(unsigned int initial_size, const std::array<double, 3>& cell_size);

    ~KDTree();

    /**
     * @brief Clear the tree
     */
    void clear();

    /**
     * @brief Insert #pose into tree and set value
     * @param pose pose to add
     * @param value value to add to the new node
     */
    void insert(const geo::Transform2& pose, double value);

    /**
     * @brief Label the current nodes by cluster
     */
    void cluster();

    /**
     * @brief Get the cluster corresponding to a pose
     * @param pose Pose to find the cluster for
     * @return cluster number, -1 if pose is not in the tree
     */
    int getCluster(const geo::Transform2& pose);

    /**
     * @brief Get the value corresponding to a pose
     * @param pose Pose to find the value for
     * @return cluster number, 0 if pose is not in the tree
     */
    double getValue(const geo::Transform2& pose);

    /**
     * @brief Get the current number of leaves
     * @return number of leaves
     */
    unsigned int getLeafCount() { return leaf_count_; }

private:

    /**
     * @brief Generate the key from a pose taking into account the #cell_size
     * @param pose pose to convert to a key
     * @return generated key
     */
    std::array<int, 3> generateKey(const geo::Transform2& pose);

    /**
     * @brief Compare keys to see if they are equal
     * @param key_a
     * @param key_b
     * @return
     */
    bool equal(const std::array<int, 3>& key_a, const std::array<int, 3>& key_b);

    /**
     * @brief Insert a node into the tree
     * @param parent parent node
     * @param node current node
     * @param key key to add
     * @param value value corresponding to the key
     * @return Pointer to the inserted node
     */
    KDTreeNode* insertNode(const KDTreeNode* parent, KDTreeNode* node, const std::array<int, 3>& key, double value);

    /**
     * @brief Recursive node search, checks if #node corresponds to #key, otherwise search in its children
     * @param node node to check
     * @param key key to get corresponding node
     * @return Pointer to the found node, nullptr if not found
     */
    KDTreeNode* findNode(KDTreeNode* node, const std::array<int, 3>& key);

    /**
     * @brief Recursively label nodes in this cluster
     * @param node node to label including its children
     * @param depth not used
     */
    void clusterNode(const KDTreeNode* node, int depth);

    // Cell size
    std::array<double, 3> res_;

    // The root node of the tree
    KDTreeNode* root_;

    // The number of nodes in the tree
    unsigned int node_count_;
    std::vector<KDTreeNode*> nodes_;

    // The number of leaf nodes in the tree
    unsigned int leaf_count_;
};

}

#endif // ED_LOCALIZATION_KDTREE_H_
