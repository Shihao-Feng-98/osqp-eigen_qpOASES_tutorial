# Osqp-Eigen_and_qpOASES
The c++ libraries for QP.

# Eigen Installation

official documents: https://eigen.tuxfamily.org/dox-3.3/group__TutorialMatrixArithmetic.html

```
sudo apt-get install libeigen3-dev
```

The default path is: `/usr/include/eigen3`. You should use

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

official repository: https://github.com/osqp/osqp

```
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIT=usr/local/osqp
sudo make install
source ~/.bashrc
```

# Osqp-eigen Installation

official repository: https://github.com/robotology/osqp-eigen

```
git clone https://github.com/robotology/osqp-eigen
cd osqp-eigen
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIT=usr/local/osqp-eigen
sudo make install
source ~/.bashrc
```

Then modify the head files in `/usr/local/include/OsqpEigen`, make sure the osqp.h can be found. Modifing the file `Constraints.hpp`, `Data.hpp`, `Settings.hpp`, `Solver.hpp`, `Solver.tpp`, `SparseMatrixHelper.hpp`.

```
#include <usr/local/include/osqp.h>
#include <usr/local/include/auxil.h>
#include <usr/local/include/scaling.h>
```

# qpOASES Installation

official repository: https://github.com/coin-or/qpOASES

```
git clone https://github.com/coin-or/qpOASES
cd qpOASES
mkdir build
cd build
cmake ..
sudo make 
sudo make install
source ~/.bashrc
```
