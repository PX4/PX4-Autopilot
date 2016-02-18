wget -O eigen.tar.bz2 http://bitbucket.org/eigen/eigen/get/3.2.8.tar.bz2
mkdir eigen
tar -xvjf eigen.tar.bz2 -C eigen --strip-components=1
mkdir eigen-build
cd eigen-build
cmake ../eigen
make
sudo make install
cd ..
mkdir Build
cd Build
cmake ../EKF
make
