
#ifndef _KD_TREE_H
#define _KD_TREE_H

#include <stddef.h>
#include <vector>

#define KDTREE_MAX_LEVEL 2
#define KDTREE_LEFT_INDEX 0
#define KDTREE_RIGHT_INDEX 1

// 节点结构
typedef struct kdnode {
        long coord_index; // 坐标索引
        float *coord; // 坐标指针
        struct kdnode *left; // 节点的左子树
        struct kdnode *right; // 节点的右子树
        int r; // 切分轴
} kdnode_t;


// knn返回结果的结构
typedef struct kdresult {
        float distance;
        long coord_index;
} kdresult_t;


// knn搜索结果列表
typedef struct knn_list {
        struct kdnode *node; // 链表头
        float distance; // 与目标点点距离
        struct knn_list *prev; // 前一列表
        struct knn_list *next; // 后一列表
} knn_list_t;

// KD-Tree结构
typedef struct kdtree {
        struct kdnode *root; // 根节点
        size_t count; // 个数
        size_t capacity; // 容量，手动更新大小，减少空间浪费
        float *coords; // 坐标
        float **coord_table; //
        long *coord_indexes; //
        unsigned char *coord_deleted; //
        unsigned char *coord_passed; // 
        struct knn_list knn_list_head; // knn搜索结果列表头
        int dim; // 维度
        int knn_num; // 当前拥有多少个knn结果
} kdtree_t;

// KD-Tree维度初始化
struct kdtree *kdtree_init(int dim);
// 节点插入
void kdtree_insert(struct kdtree *tree, float *coord);
// 构建KD-Tree
void kdtree_build(struct kdtree *tree);
// knn搜索,搜索结果在tree中的knn_list_head中
void kdtree_knn_search(struct kdtree *tree, float *target_coord, int k);
// knn结果输出
std::vector<kdresult_t> kdtree_knn_result(struct kdtree *tree);
// knn结果重置
void knn_list_reset(struct kdtree *tree);
// KD-Tree销毁
void kdtree_destroy(struct kdtree *tree);
// KD-Tree树状图可视化
void kdtree_dump(struct kdtree *tree);


#endif /* _KD_TREE_H */
