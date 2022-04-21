//
// Created by Denon on 2021/9/25.
//

#include <stdio.h>

#include "./lib/uthash.h"

struct hash_entry {
    int key;
    int val;
    UT_hash_handle hh; /* makes this structure hashable */
};

struct hash_entry *users = NULL;

// 2. 在哈希表中查找一个对象：
struct hash_entry *find_user(int key) {
    struct hash_entry *s;
    HASH_FIND_INT(users, &key, s);
    return s;
}

void add_user(int key, int val) {
    struct hash_entry *it = find_user(key);
    if (it == NULL) {
        struct hash_entry *tmp = (struct hash_entry *)malloc(sizeof(struct hash_entry));
        tmp->key = key;
        tmp->val = val;
        HASH_ADD_INT(users, key, tmp);
    } else {
        it->val = val;
    }
}

int* twoSum(int* nums, int numsSize, int target, int* returnSize) {
    users = NULL;
    for (int i = 0; i < numsSize; i++) {
        struct hash_entry *it = find_user(target - nums[i]);
        if (it != NULL) {
            int *ret = (int *)malloc(sizeof(int) * 2);
            ret[1] = i;
            ret[0] = it->val;
            *returnSize = 2;
            return ret;
        } else {
            add_user(nums[i], i);
        }
    }
    *returnSize = 0;
    return NULL;
}

int main(){
    int data_in[4] = {2,7,11,15};
    int return_n[10];
    int *result;

    result = twoSum(data_in,4,9,return_n);

    printf("ok\n");

    return 0;
}
