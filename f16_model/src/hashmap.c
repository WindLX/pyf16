#include "hashmap.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Node *create_node(const char *key, void *value) {
  Node *newNode = (Node *)malloc(sizeof(Node));
  if (!newNode) {
    error_("Memory allocation failed");
    exit(EXIT_FAILURE);
  }
  newNode->key = strdup(key);
  newNode->value = value;
  newNode->next = NULL;
  return newNode;
}

HashMap *create_hashmap(int capacity) {
  HashMap *map = (HashMap *)malloc(sizeof(HashMap));
  if (!map) {
    error_("Memory allocation failed");
    exit(EXIT_FAILURE);
  }
  map->size = 0;
  map->capacity = capacity;
  map->buckets = (Node **)calloc(capacity, sizeof(Node *));
  if (!map->buckets) {
    error_("Memory allocation failed");
    exit(EXIT_FAILURE);
  }
  return map;
}

unsigned int hash(const char *key, int capacity) {
  unsigned long hash = 0;
  while (*key) {
    hash = (hash << 4) + *key++;
    unsigned long g = hash & 0xF0000000L;
    if (g) {
      hash ^= g >> 24;
    }
    hash &= ~g;
  }
  return hash % capacity;
}

void resizeHashMap(struct HashMap *map, int newCapacity) {
  struct Node **newBuckets =
      (struct Node **)calloc(newCapacity, sizeof(struct Node *));
  if (!newBuckets) {
    perror("Memory allocation failed");
    exit(EXIT_FAILURE);
  }
  // 重新计算哈希值并重新分配元素
  for (int i = 0; i < map->capacity; ++i) {
    struct Node *current = map->buckets[i];
    while (current != NULL) {
      struct Node *next = current->next;
      unsigned int newIndex = hash(current->key, newCapacity);
      current->next = newBuckets[newIndex];
      newBuckets[newIndex] = current;
      current = next;
    }
  }
  // 释放旧的桶，并更新哈希映射的容量和桶数组
  free(map->buckets);
  map->buckets = newBuckets;
  map->capacity = newCapacity;
}

void hashmap_insert(HashMap *map, const char *key, void *value) {
  map->size += 1;
  if ((double)map->size / map->capacity > LOAD_FACTOR_THRESHOLD) {
    // 负载因子过高，执行扩容操作
    int newCapacity = map->capacity * 2;
    resizeHashMap(map, newCapacity);
  }
  unsigned int index = hash(key, map->capacity);
  Node *newNode = create_node(key, value);
  newNode->next = map->buckets[index];
  map->buckets[index] = newNode;
}

void *hashmap_get(HashMap *map, const char *key) {
  unsigned int index = hash(key, map->capacity);
  Node *current = map->buckets[index];
  while (current != NULL) {
    if (strcmp(current->key, key) == 0) {
      return current->value;
    }
    current = current->next;
  }
  return NULL;
}

void *hashmap_remove(HashMap *map, const char *key) {
  unsigned int index = hash(key, map->capacity);
  Node *current = map->buckets[index];
  Node *prev = NULL;
  while (current != NULL) {
    if (strcmp(current->key, key) == 0) {
      if (prev == NULL) {
        map->buckets[index] = current->next;
      } else {
        prev->next = current->next;
      }
      void *value = current->value;
      free(current->key);
      free(current);
      map->size -= 1;
      return value;
    }
    prev = current;
    current = current->next;
  }
  return NULL;
}

void free_hashmap(HashMap *map) {
  for (int i = 0; i < map->capacity; ++i) {
    Node *current = map->buckets[i];
    while (current != NULL) {
      Node *temp = current;
      current = current->next;
      free(temp->key);
      free(temp);
    }
  }
  free(map->buckets);
  free(map);
}