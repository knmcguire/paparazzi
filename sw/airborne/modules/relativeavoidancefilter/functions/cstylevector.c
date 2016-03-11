#include "cstylevector.h"

void vector_init(vectorstruct *v)
{
    v->capacity = VECTOR_INIT_CAPACITY;
    v->total = 0;
    // memset(v->items, 0, v->capacity);
    v->items = malloc(sizeof(void *) * v->capacity);
}

int vector_total(vectorstruct *v)
{
    return v->total;
}

void vector_resize(vectorstruct *v, int capacity)
{
    // void **s;
    // s = realloc(v->items, sizeof(void *) * capacity);
    void **items = realloc(v->items, sizeof(void *) * capacity);
    
    if (items)
    {
        v->items = items;
        v->capacity = capacity;
    }

}

void vector_add(vectorstruct *v, void *item)
{
    if (v->capacity == v->total)
    {
        vector_resize(v, v->capacity * 2);
    }
    v->items[v->total++] = item;
}

void vector_set(vectorstruct *v, int index, void *item)
{
    if (index >= 0 && index < v->total)
        v->items[index] = item;
}

void *vector_get(vectorstruct *v, int index)
{
    if (index >= 0 && index < v->total)
        return v->items[index];
    return NULL;
}

void vector_delete(vectorstruct *v, int index)
{
    int i;

    if (index < 0 || index >= v->total)
        return;

    v->items[index] = NULL;

    for (i = 0; i < v->total - 1; i++) {
        v->items[i] = v->items[i + 1];
        v->items[i + 1] = NULL;
    }

    v->total--;

    if (v->total > 0 && v->total == v->capacity / 4)
        vector_resize(v, v->capacity / 2);
}

void vector_free(vectorstruct *v)
{
    free(v->items);
}