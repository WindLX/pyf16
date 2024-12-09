#ifndef HASHMAP_H
#define HASHMAP_H

#define LOAD_FACTOR_THRESHOLD 1

typedef struct Node
{
  char *key;
  void *value;
  struct Node *next;
} Node;

typedef struct HashMap
{
  int size;
  int capacity;
  Node **buckets;
} HashMap;

Node *create_node(const char *key, void *value);

HashMap *create_hashmap(int capacity);

unsigned int hash(const char *key, int capacity);

void hashmap_insert(HashMap *map, const char *key, void *value);

void *hashmap_get(HashMap *map, const char *key);

void *hashmap_remove(HashMap *map, const char *key);

void free_hashmap(HashMap *map);

#endif