#include "bool.h"
#ifndef KEYBOARD_H
#define KEYBOARD_H

#define HIGH_SPEED        256
#define LOW_SPEED         128
#define INI_SPEED         1

#define WW                 0
#define AA                 1
#define SS                 2
#define DD                 3

typedef struct
{
    int n:1;//now
    int p:1;//previous
}keysignal_t;

void kb_stateTransfer(const keysignal_t *,const keysignal_t *, bool_t *, int *);
int kb_keyControl(const keysignal_t *,const keysignal_t *, bool_t *, int);
#endif
