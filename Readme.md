
- Install cmake
- Install Visual Studio 2019 + cpp desktop tools
- Install opencv 4.6.0 https://sourceforge.net/projects/opencvlibrary/files/4.6.0/opencv-4.6.0-vc14_vc15.exe/download
- Install boost 1.67 https://sourceforge.net/projects/boost/files/boost-binaries/1.67.0/
- ``mkdir Build``
- ``cd Build``
- ``cmake -G "Visual Studio 16 2019" -A x64 -D OpenCV_DIR="C:\path\to\opencv\build" -D Boost_INCLUDE_DIR="C:\local\boost_1_67_0" ..``
- open sln
- it should be able to build mageslam_c_api now


---

## Building a wasm

### Install emscripten
- install emscripten: https://chocolatey.org/packages/emscripten via https://emscripten.org/docs/getting_started/downloads.html

### Build opencv for unix
- download https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.4.3/
- ``mkdir build && cd build``
- ``cmake -G "Unix Makefiles" ..``

### Include boost
- Add ``include_directories("Shared" "C:/local/boost_1_67_0")`` to CMakeList

### Update eigen
- Commit sha: 36b95962756c1fce8e29b1f8bc45967f30773c00 fixing error ``class template partial specialization is not more specialized than the primary template [-Winvalid-partial-specialization]``

### Generate and run build
- run these two commands in the Build directory
```
emcmake cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=MinSizeRel -D OpenCV_DIR="C:\Users\marce\Downloads\unix\opencv-3.4.3\opencv-3.4.3\build" ..
cmake --build .
```

- ~~compile mageslam_c_api to wasm using ``emcc --emit-symbol-map mageslam_c_api_d.lib``~~
https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.4.3/opencv-3.4.3.zip/download

## Troubleshooting

#### "boost can not be found" when building
- add additional c++ include directories in Arcana and MAGESLAM projects (the projects that throw the error) to ``C:\local\boost_1_67_0``



## Other links

- https://github.com/floooh/pacman.c
- Online WASM to text: https://webassembly.github.io/wabt/demo/wasm2wat/