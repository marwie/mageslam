#include <stdio.h>



#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif

EXTERN void myFunction(int argc) {
    printf("MyFunction Called\n");
}


int main() {
    //printf("Hello World 123\n");
    myFunction(0);
    return 0;
}
