
- Install cmake
- Install Visual Studio 2019 + cpp desktop tools
- Install opencv 4.6.0 https://sourceforge.net/projects/opencvlibrary/files/4.6.0/opencv-4.6.0-vc14_vc15.exe/download
- Install boost 1.67 https://sourceforge.net/projects/boost/files/boost-binaries/1.67.0/
- ``mkdir Build``
- ``cd Build``
- ``cmake -G "Visual Studio 16 2019" -A x64 -D OpenCV_DIR="C:\path\to\opencv\build" -D Boost_INCLUDE_DIR="C:\local\boost_1_67_0 .."``
- open sln
- it should be able to build mageslam_c_api now

- after that install emscripten: https://chocolatey.org/packages/emscripten via https://emscripten.org/docs/getting_started/downloads.html
- compile mageslam_c_api to wasm using ``emcc --emit-symbol-map mageslam_c_api_d.lib``

## Troubleshooting

#### "boost can not be found" when building
- add additional c++ include directories in Arcana and MAGESLAM projects (the projects that throw the error) to ``C:\local\boost_1_67_0``

