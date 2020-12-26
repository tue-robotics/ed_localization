#include "kdtree.h"
#include "transform.h"

#include <cmath>

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------


KDTree::KDTree(unsigned int initial_size, const std::array<double, 3>& cell_size) : res_(cell_size), root_(nullptr), node_count_(0), nodes_(initial_size), leaf_count_(0)
{
}

// ----------------------------------------------------------------------------------------------------

KDTree::~KDTree()
{
    for (auto ptr : nodes_)
        delete ptr;
}

// ----------------------------------------------------------------------------------------------------

void KDTree::clear()
{
    root_ = nullptr;
    leaf_count_ = 0;
    node_count_ = 0;
}

// ----------------------------------------------------------------------------------------------------

void KDTree::insert(const Transform& pose, double value)
{
    root_ = insertNode(nullptr, root_, generateKey(pose), value);
}

// ----------------------------------------------------------------------------------------------------

void KDTree::cluster()
{
    unsigned int queue_count = 0;
    unsigned int cluster_count = 0;

    std::vector<KDTreeNode*> queue(node_count_);

    // Put all the leaves in a queue
    for (uint i=0; i<node_count_; ++i)
    {
        KDTreeNode* node = nodes_[i];
        if (node->leaf)
        {
            node->cluster = -1;
            queue[queue_count++] = node;
        }
    }

    for (uint i=queue_count; i>0; --i)
    {
        KDTreeNode* node = queue[i-1];

        // If this node has already been labelled, skip it
        if (node->cluster >= 0)
            continue;

        // Assign a label to this cluster
        node->cluster = cluster_count++;

        // Recursively label nodes in this cluster
        clusterNode(node, 0);
    }
}

// ----------------------------------------------------------------------------------------------------

int KDTree::getCluster(const Transform& pose)
{
    KDTreeNode* node = findNode(root_, generateKey(pose));
    if (!node)
        return -1;
    return node->cluster;
}

// ----------------------------------------------------------------------------------------------------

double KDTree::getValue(const Transform& pose)
{
    KDTreeNode* node = findNode(root_, generateKey(pose));
    if (!node)
        return 0;
    return node->value;
}

// ----------------------------------------------------------------------------------------------------

std::array<int, 3> KDTree::generateKey(const Transform &pose)
{
    std::array<int, 3> key;
    key[0] = std::floor(pose.translation().x / res_[0]);
    key[1] = std::floor(pose.translation().y / res_[1]);
    key[2] = std::floor(pose.rotation() / res_[2]);

    return key;
}

// ----------------------------------------------------------------------------------------------------

bool KDTree::equal(const std::array<int, 3>& key_a, const std::array<int, 3>& key_b)
{
    if (key_a[0] != key_b[0])
      return false;
    if (key_a[1] != key_b[1])
      return false;
    if (key_a[2] != key_b[2])
      return false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

KDTreeNode* KDTree::insertNode(const KDTreeNode* parent, KDTreeNode* node, const std::array<int, 3>& key, double value)
{
    // If the node doesnt exist yet
    if (!node)
    {
        // Resize if end is reached
        if (node_count_ >= nodes_.size())
            nodes_.resize(node_count_ + 10);


        if (nodes_[node_count_])
            nodes_[node_count_] = new(nodes_[node_count_]) KDTreeNode;
        else
            nodes_[node_count_] = new KDTreeNode;
        node = nodes_[node_count_++];
        node->leaf = true;

        if (!parent)
            node->depth = 0;
        else
            node->depth = parent->depth + 1;

        node->key = key;
        node->value = value;
        leaf_count_ += 1;
    }

    // If the node exists, and it is a leaf node
    else if (node->leaf)
    {
        // If the keys are equal, increment the value
        if (equal(key, node->key))
        {
            node->value += value;
        }

        // The keys are not equal, so split this node
        else
        {
            // Pivot dimension should be x, y, th, x, y, th, etc.
            node->pivot_dim = node->depth % 3;

            node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.;

            if (key[node->pivot_dim] < node->pivot_value)
            {
              node->children[0] = insertNode(node, nullptr, key, value);
              node->children[1] = insertNode(node, nullptr, node->key, node->value);
            }
            else
            {
              node->children[0] = insertNode(node, nullptr, node->key, node->value);
              node->children[1] = insertNode(node, nullptr, key, value);
            }

            node->leaf = 0;
            leaf_count_ -= 1;
        }
    }

    // If the node exists and it has children
    else
    {
        if (key[node->pivot_dim] < node->pivot_value)
            insertNode(node, node->children[0], key, value);
        else
            insertNode(node, node->children[1], key, value);
    }

    return node;
}

// ----------------------------------------------------------------------------------------------------

KDTreeNode* KDTree::findNode(KDTreeNode* node, const std::array<int, 3>& key)
{
    if (node->leaf)
    {
        // If the keys are the same
        if (equal(key, node->key))
            return node;
        else
            return nullptr;
    }
    else
    {
        // If the keys are different
        if (key[node->pivot_dim] < node->pivot_value)
            return findNode(node->children[0], key);
        else
            return findNode(node->children[1], key);
    }
}

// ----------------------------------------------------------------------------------------------------

void KDTree::clusterNode(const KDTreeNode* node, int depth)
{
    std::array<int, 3> nkey;
    KDTreeNode* nnode;
    for (uint i=0; i<27; ++i) // all surrounding bins, including yourself
    {
        nkey[0] = node->key[0] + (i / 9) - 1;
        nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
        nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

        nnode = findNode(root_, nkey);
        if (!nnode)
            continue;

        // This node already has a label, skip it.
        if (nnode->cluster >= 0)
            continue;

        // Label this node and recurse
        nnode->cluster = node->cluster;
        clusterNode(nnode, depth + 1);
    }
}
