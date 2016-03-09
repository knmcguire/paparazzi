#ifndef CSTYLEVECTOR_H
#define CSTYLEVECTOR_H

#include "stdlib.h"

#define VECTOR_INIT_CAPACITY 6

/*
Compliments of Edd Mann
http://eddmann.com/posts/implementing-a-dynamic-vector-array-in-c/
Visited 20 Jan 2014
*/

typedef struct vectorstruct {
    void **items;
    int capacity;
    int total;
} vectorstruct;

void vector_init(vectorstruct *);
int vector_total(vectorstruct *);
void vector_resize(vectorstruct *, int);
void vector_add(vectorstruct *, void *);
void vector_set(vectorstruct *, int, void *);
void *vector_get(vectorstruct *, int);
void vector_delete(vectorstruct *, int);
void vector_free(vectorstruct *);

#endif