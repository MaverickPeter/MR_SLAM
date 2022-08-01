#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#include "global_manager/kdtree.h"

// static inline 确保编译器会执行内联操作，而不仅仅是建议

/** 是否是叶节点
 * @param
 * @return
 */
static inline int is_leaf(struct kdnode *node)
{
        return node->left == node->right;
}

/** 交换数据
 * @param a
 * @param b
 */
static inline void swap(long *a, long *b)
{
        long tmp = *a;
        *a = *b;
        *b = tmp;
}

/** 距离平方
 * @param
 * @return
 */
static inline float square(float d)
{
        return d * d;
}

/** 距离大小,1范式(各个维度的距离差平方之和)
 * @param
 * @param
 * @param
 * @return
 */
static inline float distance(float *c1, float *c2, int dim)
{
        float distance = 0;
        while (dim-- > 0) {
                distance += square(*c1++ - *c2++);
        }
        return distance;
}

/** 记录knn结果中最大距离
 * @param
 * @return
 */
static inline float knn_max(struct kdtree *tree)
{
        return tree->knn_list_head.prev->distance;
}

/** 取值操作
 * @param kdtree
 * @param index
 * @param r
 * @return 指定坐标位置
 */
static inline float D(struct kdtree *tree, long index, int r)
{
        return tree->coord_table[index][r];
}

/** 改点是否需要跳过，若是的话，则返回1; 若为空也返回1
 * @param kdtree 
 * @param node
 * @return 1 or 0
 */
static inline int kdnode_passed(struct kdtree *tree, struct kdnode *node)
{
        return node != NULL ? tree->coord_passed[node->coord_index] : 1;
}

/**
 * @param kdtree
 * @param k
 * @param 当前点
 * @param 目标
 * @return  还未到达上限(k)或者小于最大距离
 */
static inline int knn_search_on(struct kdtree *tree, int k, float value, float target)
{
        return tree->knn_num < k || square(target - value) < knn_max(tree);
}

/** 为数据的索引做初始化
 * @param kdtree
 */
static inline void coord_index_reset(struct kdtree *tree)
{
        long i;
        for (i = 0; i < tree->capacity; i++) {
                tree->coord_indexes[i] = i;
        }
}

/** 点集的坐标表管理
 * @param kdtree
 */
static inline void coord_table_reset(struct kdtree *tree)
{
        long i;
        for (i = 0; i < tree->capacity; i++) {
                tree->coord_table[i] = tree->coords + i * tree->dim; // 首地址+index*increment
                // 指针的指向位置为对应的坐标在内存中的位置
        }
}

/** 重置为0
 * @param kdtree
 */
static inline void coord_deleted_reset(struct kdtree *tree)
{       // 清空数组
        memset(tree->coord_deleted, 0, sizeof(unsigned char)*tree->capacity);
}

/** 重置为0
 * @param kdtree
 */
static inline void coord_passed_reset(struct kdtree *tree)
{
        memset(tree->coord_passed, 0, sizeof(unsigned char)*tree->capacity);
}


/** 插入排序
 * @param kdtree
 * @param low排序列表起点
 * @param high排序列表终点
 * @param r切分轴
 */
static void insert_sort(struct kdtree *tree, long low, long high, int r)
{
        long i, j;
        long *indexes = tree->coord_indexes;
        for (i = low + 1; i <= high; i++) {
                long tmp_idx = indexes[i];
                float tmp_value = D(tree, indexes[i], r);
                j = i - 1;
                // 从小到大的顺序排列
                for (; j >= low && D(tree, indexes[j], r) > tmp_value; j--) {
                        indexes[j + 1] = indexes[j];
                }
                indexes[j + 1] = tmp_idx;
        }
}

/** 根据r所规定的轴进行高效率的排序
 * @param kdtree
 * @param low排序列表起点
 * @param high排序列表终点
 * @param r切分轴
 */
static void quicksort(struct kdtree *tree, long low, long high, int r)
{       
        // 若数量较少，采用插入排序，否则采用快排
        if (high - low <= 32) {
                insert_sort(tree, low, high, r);
                //bubble_sort(tree, low, high, r);
                return;
        }

        long *indexes = tree->coord_indexes;
        /* median of 3 */
        long mid = low + (high - low) / 2;
        if (D(tree, indexes[low], r) > D(tree, indexes[mid], r)) {
                swap(indexes + low, indexes + mid);
        }
        if (D(tree, indexes[low], r) > D(tree, indexes[high], r)) {
                swap(indexes + low, indexes + high);
        }
        if (D(tree, indexes[high], r) > D(tree, indexes[mid], r)) {
                swap(indexes + high, indexes + mid);
        }

        /* D(indexes[low]) <= D(indexes[high]) <= D(indexes[mid]) */
        float pivot = D(tree, indexes[high], r);

        /* 3-way partition
         * +---------+-----------+---------+-------------+---------+
         * |  pivot  |  <=pivot  |   ?     |  >=pivot    |  pivot  |
         * +---------+-----------+---------+-------------+---------+
         * low     lt             i       j               gt    high
         */
        long i = low - 1;
        long lt = i;
        long j = high;
        long gt = j;
        for (; ;) {
                while (D(tree, indexes[++i], r) < pivot) {}
                while (D(tree, indexes[--j], r) > pivot && j > low) {}
                if (i >= j) break;
                swap(indexes + i, indexes + j);
                if (D(tree, indexes[i], r) == pivot) swap(&indexes[++lt], &indexes[i]);
                if (D(tree, indexes[j], r) == pivot) swap(&indexes[--gt], &indexes[j]);
        }
        /* i == j or j + 1 == i */
        swap(indexes + i, indexes + high);

        /* Move equal elements to the middle of array */
        long x, y;
        for (x = low, j = i - 1; x <= lt && j > lt; x++, j--) swap(indexes + x, indexes + j);
        for (y = high, i = i + 1; y >= gt && i < gt; y--, i++) swap(indexes + y, indexes + i);

        quicksort(tree, low, j - lt + x - 1, r);
        quicksort(tree, i + y - gt, high, r);
}

/** 通过指针方式，减少数据拷贝复制，所以最后生成的kdtree就是一个个指针构建的树，指针指向本地的表
 * @param 坐标在表中的地址
 * @param 坐标索引
 * @param 切分轴
 * @return 节点指针
 */
static struct kdnode *kdnode_alloc(float *coord, long index, int r)
{
        struct kdnode *node = static_cast<kdnode*>(malloc(sizeof(*node)));
        if (node != NULL) {
                memset(node, 0, sizeof(*node));
                node->coord = coord;
                node->coord_index = index;
                node->r = r;
        }
        return node;
}

static void kdnode_free(struct kdnode *node)
{
        free(node);
}

/** 比较两节点，c1>c2,则返回1
 * @param
 * @param
 * @param
 * @return
 */
static int coord_cmp(float *c1, float *c2, int dim)
{
        int i;
        float ret;
        for (i = 0; i < dim; i++) {
                ret = *c1++ - *c2++;
                if (fabs(ret) >= DBL_EPSILON) {
                        return ret > 0 ? 1 : -1;
                }
        }

        if (fabs(ret) < DBL_EPSILON) {
                return 0;
        } else {
                return ret > 0 ? 1 : -1;
        }
}

/** 将新节点加入knn列表，并保持升序排列
 * @param
 * @param
 * @param
 */
static void knn_list_add(struct kdtree *tree, struct kdnode *node, float distance)
{
        if (node == NULL) return;

        struct knn_list *head = &tree->knn_list_head;
        struct knn_list *p = head->prev;
        // 将指针指向 适合新晋节点的位置（即前向节点的距离小，后向节点的距离大）
        if (tree->knn_num == 1) {
                if (p->distance > distance) {
                        p = p->prev;
                }
        } else {
                while (p != head && p->distance > distance) {
                        p = p->prev;
                }
        }

        // 第一个点时 或 list中当前指向的节点与新插入节点不等时, 加入node
        if (p == head || coord_cmp(p->node->coord, node->coord, tree->dim)) {
                struct knn_list *log = static_cast<knn_list*>(malloc(sizeof(*log)));
                if (log != NULL) {
                        log->node = node;
                        log->distance = distance;
                        log->prev = p;
                        log->next = p->next;
                        p->next->prev = log;
                        p->next = log;
                        tree->knn_num++;
                }
        }
}

/** 将新晋节点替换最远距离的节点
 * @param
 * @param
 * @param
 */
static void knn_list_adjust(struct kdtree *tree, struct kdnode *node, float distance)
{
        if (node == NULL) return;

        struct knn_list *head = &tree->knn_list_head;
        struct knn_list *p = head->prev;
        if (tree->knn_num == 1) {
                if (p->distance > distance) {
                        p = p->prev;
                }
        } else {
                while (p != head && p->distance > distance) {
                        p = p->prev;
                }
        }

        if (p == head || coord_cmp(p->node->coord, node->coord, tree->dim)) {
                struct knn_list *log = head->prev;
                /* Replace the original max one */
                log->node = node;
                log->distance = distance;
                /* Remove from the max position */
                head->prev = log->prev;
                log->prev->next = head;
                /* insert as a new one */
                log->prev = p;
                log->next = p->next;
                p->next->prev = log;
                p->next = log;
        }
}

/** 将结果清空
 * @param
 */
static void knn_list_clear(struct kdtree *tree)
{
        struct knn_list *head = &tree->knn_list_head;
        struct knn_list *p = head->next;
        while (p != head) {
                struct knn_list *prev = p;
                p = p->next;
                free(prev);
        }
        tree->knn_num = 0;
}

/** 重置结果,即重置双向回环链表
 * @param
 */
void knn_list_reset(struct kdtree *tree){

    knn_list_clear(tree);
    tree->knn_list_head.next = &tree->knn_list_head;
    tree->knn_list_head.prev = &tree->knn_list_head;
    tree->knn_list_head.node = NULL;
    tree->knn_list_head.distance = 0;
    tree->knn_num = 0;
    coord_passed_reset(tree);
}

/** 为kdtree扩容(2倍)
 * @param kdtree
 */
static void resize(struct kdtree *tree)
{
        tree->capacity *= 2;
        tree->coords = static_cast<float*>(realloc(tree->coords, tree->dim * sizeof(float) * tree->capacity));
        tree->coord_table = static_cast<float**>(realloc(tree->coord_table, sizeof(float *) * tree->capacity));
        tree->coord_indexes = static_cast<long*>(realloc(tree->coord_indexes, sizeof(long) * tree->capacity));
        tree->coord_deleted = static_cast<unsigned char*>(realloc(tree->coord_deleted, sizeof(unsigned char) * tree->capacity));
        tree->coord_passed = static_cast<unsigned char*>(realloc(tree->coord_passed, sizeof(unsigned char) * tree->capacity));
        coord_table_reset(tree);
        coord_index_reset(tree);
        coord_deleted_reset(tree);
        coord_passed_reset(tree);
}

/** kdtree初始化
 * @param dim:维度
 * @return kdtree结构体指针
 */
struct kdtree *kdtree_init(int dim)
{
  // 分配一个kdtree struct的内存空间,但不会初始化赋值（失败的话会返回NULL）
  struct kdtree *tree = static_cast<kdtree*>(malloc(sizeof(*tree)));
  if (tree != NULL) {
    tree->root = NULL;
    tree->dim = dim;
    tree->count = 0;
    tree->capacity = 65536; //2字节的大小,16位
    tree->knn_list_head.next = &tree->knn_list_head;
    tree->knn_list_head.prev = &tree->knn_list_head;
    tree->knn_list_head.node = NULL;
    tree->knn_list_head.distance = 0;
    tree->knn_num = 0;
    tree->coords = static_cast<float*>(malloc(dim * sizeof(float) * tree->capacity));
    tree->coord_table = static_cast<float**>(malloc(sizeof(float *) * tree->capacity));
    tree->coord_indexes = static_cast<long*>(malloc(sizeof(long) * tree->capacity));
    tree->coord_deleted = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * tree->capacity));
    tree->coord_passed = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * tree->capacity));
    coord_index_reset(tree);
    coord_table_reset(tree);
    coord_deleted_reset(tree);
    coord_passed_reset(tree);
  }
  return tree;
}

/** 插入新的节点至表中
 * @param kdtree指针
 * @param 坐标指针
 */
void kdtree_insert(struct kdtree *tree, float *coord)
{
        // 扩容
        if (tree->count + 1 > tree->capacity) {
                resize(tree);
        }
        // 通过内存拷贝，将指定地址中的坐标信息拷贝到坐标表中
        memcpy(tree->coord_table[tree->count++], coord, tree->dim * sizeof(float));
}

/** 为kdtree构建节点
 * @param kdtree
 * @param nptr 当前节点
 * @param r 切分轴
 * @param low 列表起始
 * @param high 列表末端
 */
static void kdnode_build(struct kdtree *tree, struct kdnode **nptr, int r, long low, long high)
{
  // 结束条件,即分支上只剩下一个节点
  if (low == high) {
    long index = tree->coord_indexes[low];
    *nptr = kdnode_alloc(tree->coord_table[index], index, r);
  } else if (low < high) {
    /* Sort and fetch the median to test a balanced BST */
    quicksort(tree, low, high, r);
    long median = low + (high - low) / 2;
    long median_index = tree->coord_indexes[median];// 通过给索引排序，减少数据的移动
    struct kdnode *node = *nptr = kdnode_alloc(tree->coord_table[median_index], median_index, r);
    r = (r + 1) % tree->dim;
    kdnode_build(tree, &node->left, r, low, median - 1);
    kdnode_build(tree, &node->right, r, median + 1, high);
  }
}

/** 构建kdtree
 * @param kdtree
 */
void kdtree_build(struct kdtree *tree)
{
  kdnode_build(tree, &tree->root, 0, 0, tree->count - 1);
}

/** 将符合条件的node加入knn结果
 * @param
 * @param
 * @param
 * @param
 */
static void knn_pickup(struct kdtree *tree, struct kdnode *node, float *target, int k)
{
        float dist = distance(node->coord, target, tree->dim);
        if (tree->knn_num < k) {
                knn_list_add(tree, node, dist);
        } else {
                if (dist < knn_max(tree)) {
                        knn_list_adjust(tree, node, dist);
                } else if (fabs(dist - knn_max(tree)) < DBL_EPSILON) { // float_Epsilon双精度最小误差
                        knn_list_add(tree, node, dist);
                }
        }
}

/** 迭代寻找临近点
 * @param kdtree
 * @param 当前节点
 * @param 目标
 * @param k
 * @param 
 */
static void kdtree_search_recursive(struct kdtree *tree, struct kdnode *node, float *target, int k, int *pickup)
{
        // 当节点到底或者被已被搞定
        if (node == NULL || kdnode_passed(tree, node)) {
                return;
        }

        // 当列表已满且无法更新插入
        int r = node->r;
        if (!knn_search_on(tree, k, node->coord[r], target[r])) {
                return;
        }

        if (*pickup) {
                tree->coord_passed[node->coord_index] = 1;
                knn_pickup(tree, node, target, k);
                kdtree_search_recursive(tree, node->left, target, k, pickup);
                kdtree_search_recursive(tree, node->right, target, k, pickup);
        } else {
                if (is_leaf(node)) {
                        *pickup = 1; // 若是叶节点，标记处理过
                } else {
                        if (target[r] <= node->coord[r]) {
                                kdtree_search_recursive(tree, node->left, target, k, pickup);
                                kdtree_search_recursive(tree, node->right, target, k, pickup);
                        } else {
                                kdtree_search_recursive(tree, node->right, target, k, pickup);
                                kdtree_search_recursive(tree, node->left, target, k, pickup);
                        }
                }
                /* back track and pick up  */
                if (*pickup) {
                        tree->coord_passed[node->coord_index] = 1; //把它处理掉
                        knn_pickup(tree, node, target, k);
                }
        }
}

/** 根据目标进行临近点搜索，结果存储在kdtree中的knn_list中
 * @param kdtree
 * @param target
 * @param k
 */
void kdtree_knn_search(struct kdtree *tree, float *target, int k)
{
        if (k > 0) {
                int pickup = 0;
                kdtree_search_recursive(tree, tree->root, target, k, &pickup);
        }
}

// // 输出显示
// void kdtree_knn_result(struct kdtree *tree)
// {
//   int i;
//   struct knn_list *p = tree->knn_list_head.next;
//   while (p != &tree->knn_list_head) {
//     putchar('(');
//     for (i = 0; i < tree->dim; i++) {
//       if (i == tree->dim - 1) {
//         printf("%.2lf) Distance:%lf, Index: %ld\n", p->node->coord[i], sqrt(p->distance), p->node->coord_index);
//       } else {
//         printf("%.2lf, ", p->node->coord[i]);
//       }
//     }
//     p = p->next;
//   }
// }

std::vector<kdresult_t> kdtree_knn_result(struct kdtree *tree)
{
  int i;
  std::vector<kdresult_t> output;
  struct knn_list *p = tree->knn_list_head.next;
  while (p != &tree->knn_list_head) {
//     putchar('(');
    for (i = 0; i < tree->dim; i++) {
      if (i == tree->dim - 1) {
        // printf("Distance:%lf, Index: %ld\n", sqrt(p->distance), p->node->coord_index);
        kdresult_t result;
        result.coord_index = p->node->coord_index;
        result.distance = sqrt(p->distance);
        output.emplace_back(result);
      } else {
        // kdresult_t result;
        // result.coord_index = p->node->coord_index;
        // result.distance = sqrt(p->distance);
        // output.emplace_back(result);
        // printf("%.2lf, ", p->node->coord[i]);
      }
    }
    p = p->next;
  }
  return output;
}


/** 清空节点内存
 * @param
 */
static void kdnode_destroy(struct kdnode *node)
{
        if (node == NULL) return;
        kdnode_destroy(node->left);
        kdnode_destroy(node->right);
        kdnode_free(node);
}

/** 清空kdtree内存
 * @param
 */
void kdtree_destroy(struct kdtree *tree)
{
        kdnode_destroy(tree->root);
        knn_list_clear(tree);
        free(tree->coords);
        free(tree->coord_table);
        free(tree->coord_indexes);
        free(tree->coord_deleted);
        free(tree->coord_passed);
        free(tree);
}

/** 用于可视化的节点结构体
 */
struct kdnode_backlog {
        struct kdnode *node;
        int next_sub_idx;
};


/** 输出节点
 * @param node 节点
 * @param dim 维度
 */
static void kdnode_dump(struct kdnode *node, int dim)
{
  int i;
  if (node->coord != NULL) {
    printf("(");
    for (i = 0; i < dim; i++) {
      if (i != dim - 1) {
        printf("%.2f,", node->coord[i]);
      } else {
        printf("%.2f)\n", node->coord[i]);
      }
    }
  } else {
    printf("(none)\n");
  }
}


/** 输出kdtree树状图
 * @param
 */
void kdtree_dump(struct kdtree *tree)
{
        int level = 0;
        struct kdnode *node = tree->root;
        struct kdnode_backlog nbl, *p_nbl = NULL;
        struct kdnode_backlog nbl_stack[KDTREE_MAX_LEVEL];
        struct kdnode_backlog *top = nbl_stack;

        for (; ;) {
                if (node != NULL) {
                        /* Fetch the pop-up backlogged node's sub-id.
                         * If not backlogged, fetch the first sub-id. */
                        int sub_idx = p_nbl != NULL ? p_nbl->next_sub_idx : KDTREE_RIGHT_INDEX;

                        /* Backlog should be left in next loop */
                        p_nbl = NULL;

                        /* Backlog the node */
                        if (is_leaf(node) || sub_idx == KDTREE_LEFT_INDEX) {
                                top->node = NULL;
                                top->next_sub_idx = KDTREE_RIGHT_INDEX;
                        } else {
                                top->node = node;
                                top->next_sub_idx = KDTREE_LEFT_INDEX;
                        }
                        top++;
                        level++;

                        /* Draw lines as long as sub_idx is the first one */
                        if (sub_idx == KDTREE_RIGHT_INDEX) {
                                int i;
                                for (i = 1; i < level; i++) {
                                        if (i == level - 1) {
                                                printf("%-8s", "+-------");
                                        } else {
                                                if (nbl_stack[i - 1].node != NULL) {
                                                        printf("%-8s", "|");
                                                } else {
                                                        printf("%-8s", " ");
                                                }
                                        }
                                }
                                kdnode_dump(node, tree->dim);
                        }

                        /* Move down according to sub_idx */
                        node = sub_idx == KDTREE_LEFT_INDEX ? node->left : node->right;
                } else {
                        p_nbl = top == nbl_stack ? NULL : --top;
                        if (p_nbl == NULL) {
                                /* End of traversal */
                                break;
                        }
                        node = p_nbl->node;
                        level--;
                }
        }
}
