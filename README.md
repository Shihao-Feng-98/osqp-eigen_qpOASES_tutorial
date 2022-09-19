# Osqp-Eigen_and_qpOASES
The c++ libraries for QP.

# Eigen Installation

```
sudo apt-get install libeigen3-dev
```

The default path is: /usr/include/eigen3. You should use

```
#include <eigen3/Eigen/Dense>
```

or you can 

```
sudo cp -r  /usr/include/eigen3/Eigen /usr/include/
```

then use 

```
#include <Eigen/Dense>
```

# OSQP Installation

Official repository https://github.com/osqp/osqp

```
git clone https://github.com/osqp/osqp
cd osqp
mkdir build
cmake ..
cmake .. -DCMAKE_INSTALL_PREFIT=usr/local/osqp
sudo make install
source ~/.bashrc
```

