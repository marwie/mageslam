#include <stdio.h>


#include <emscripten.h>



#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif


EXTERN EMSCRIPTEN_KEEPALIVE void myFunction(int argc) {
    printf("MyFunction Called\n");
}


int main() {
    return 0;
}
