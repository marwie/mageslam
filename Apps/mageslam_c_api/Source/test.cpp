#include <stdio.h>


#include <emscripten.h>


#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif


EXTERN EMSCRIPTEN_KEEPALIVE void NEEDLE123() {
    printf("MyFunction Called\n");
}

