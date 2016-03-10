#ifndef CSTYLEVECTOR_H
#define CSTYLEVECTOR_H

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

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

extern void vector_init(vectorstruct *);
extern int vector_total(vectorstruct *);
extern void vector_resize(vectorstruct *, int);
extern void vector_add(vectorstruct *, void *);
extern void vector_set(vectorstruct *, int, void *);
extern void *vector_get(vectorstruct *, int);
extern void vector_delete(vectorstruct *, int);
extern void vector_free(vectorstruct *);

#endif