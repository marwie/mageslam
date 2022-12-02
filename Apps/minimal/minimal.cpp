#include <stdio.h>


#include <emscripten/emscripten.h>


#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif


EXTERN EMSCRIPTEN_KEEPALIVE void myFunction(int argc) {
    printf("MyFunction Called\n");
}


int main() {
    printf("Hello World 123\n");
    return 0;
}
