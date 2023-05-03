rm -rf libs && mkdir libs && cd libs

#C/C++/python3
sudo apt install -y build-essential python3 python-is-python3 libpython3-dev python3-pip 

#cmake
sudo apt install -y gpg wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
sudo apt-get install -y kitware-archive-keyring
sudo apt update
sudo apt upgrade -y cmake
cmake --version

#boost   
sudo apt install -y libboost-all-dev
#eigen   
sudo apt install -y libeigen3-dev 
#numpy
python -m pip install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple/ 

#blasfeo
git clone https://github.com/giaf/blasfeo.git blasfeo
cd blasfeo/ && mkdir build 
cd build && cmake .. && make -j8 && sudo make install
cd ../../

#cppad
git clone https://github.com/coin-or/CppAD.git cppad
cd cppad && mkdir build 
cd build && cmake .. && make check -j8 && sudo make install
cd ../../

#CppADCodeGen
sudo apt install -y libgtest-dev clang lldb lld valgrind
git clone https://github.com/joaoleal/CppADCodeGen.git CppADCodeGen
cd CppADCodeGen/ && mkdir build
cd build && cmake .. && sudo make install
cd ../../

#hpipm
git clone https://github.com/giaf/hpipm.git hpipm
cd hpipm && mkdir build
cd build && cmake .. && make -j8 && sudo make install
cd ../../

#kindr
git clone https://github.com/ANYbotics/kindr.git kindr
cd kindr && mkdir build
cd build && cmake .. && make -j8 && sudo make install
cd ../../

#control-toolbox 
sudo apt-get install -y liblapack-dev 
git clone https://github.com/ethz-adrl/control-toolbox.git
cd control-toolbox/ct
chmod 775 build_ct.sh
./build_ct.sh -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true


# add into control_toolbox/ct_models/CMakeLists.txt: 22
# 
# ## include blasfeo and hpipm, assumed to be installed in "/opt"
# list(APPEND CMAKE_PREFIX_PATH "/opt")
# find_package(blasfeo QUIET)
# find_package(hpipm QUIET)
# if(blasfeo_FOUND AND hpipm_FOUND)
#     message(STATUS "Found HPIPM and BLASFEO")
#     set(HPIPM ON)
#     list(APPEND HPIPM_LIBS hpipm blasfeo)
#     list(APPEND ct_optcon_COMPILE_DEFINITIONS HPIPM)
# else()
#     message(WARNING "Could not find HPIPM or BLASFEO")
# endif()
