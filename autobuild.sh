mkdir -p build
cd build
cmake -DCMAKE_CXX_FLAGS="-W0" -G "Visual Studio 17 2022" -A x64 ..
# cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release --verbose --parallel 8

