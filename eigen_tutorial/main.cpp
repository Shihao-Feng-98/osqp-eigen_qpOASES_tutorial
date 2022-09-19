#include<iostream>
using namespace std;
#include<Eigen/Dense> 
using namespace Eigen;

void test01()
{
    MatrixXd m(2,2);
    m(0,0) = 1;
    m(1,0) = 2;
    m(0,1) = 3;
    m(1,1) = m(0,0) + m(0,1);
    cout << "m = " << endl << m << endl;
    VectorXd v(2);
    v(0) = 4;
    v(1) = v(0) - 1;
    cout << "v = " << endl << v << endl;
}

void test02()
{
    // 运行时确定矩阵维度
    MatrixXd m = MatrixXd::Random(3,3);
    m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
    cout << "m = " << endl << m << endl;
    VectorXd v(3);
    v << 1, 2, 3; // 逗号初始化
    cout << "m * x = " << endl << m * v << endl;
}

void test03()
{
    // 编译时确定矩阵维度
    Matrix3d m = Matrix3d::Random(); // [-1,1]随机取值
    m = (m + Matrix3d::Constant(1.2)) * 50; // [0.2,2.2]*50
    cout << "m = " << endl << m << endl;
    Vector3d v(1,2,3); // 有参构造 column vector
    cout << "m * x = " << endl << m * v << endl;
}

void test04()
{
    // 初始化
    Matrix<int, 3, 1> a{1,2,3};
    // Matrix<double, 1, 4> b = {1.1, 2.2, 3.3, 4.4};
    Vector2i c(4,5);
    Matrix3f d;
    d << 1,2,3, // 逗号初始化
        4,5,6,
        7,8,9;
    cout << d << endl;
    Vector3f e;
    e << 1.1, 2.2, 3.3;
    // 已有的相邻拼接
    VectorXf joined(6);
    joined << e, e;
    cout << joined << endl;
    // 已有的矩阵拼接
    Matrix2d matA; matA << 1, 2, 3, 4;
    Matrix4d matB;
    matB << matA, matA/10, matA/5, matA;
    cout << matB << endl;
    // 按块赋值
    Matrix3f m;
    m.row(0) << 1, 2, 3;
    m.block(1,0,2,2) << 4, 5, 7, 8;
    m.col(2).tail(2) << 6, 9;                   
    cout << m << endl;
}

void test05()
{
    // size() rows() cols() 返回矩阵大小 和 维度
    MatrixXd m(2,5);
    m.resize(4,3);
    cout << "Matrix is of size " << m.rows() << "x" << m.cols() << endl;
    cout << "It has " << m.size() << " coefficients" << endl;
    VectorXd v(2);
    v.resize(5);
    cout << "Vector is of size " << v.size() << endl;
    cout << "As matrix, it is of size " << v.rows() << "x" << v.cols() << endl;
}

void test06()
{
    // 没有给定矩阵大小时，可以通过赋值重新改变矩阵大小
    MatrixXf a(2,2);
    cout << "Matrix is of size " << a.rows() << "x" << a.cols() << endl;
    MatrixXf b(3,3);
    a = b;
    cout << "Matrix is now of size " << a.rows() << "x" << a.cols() << endl;
}

void test07()
{
    // Matrix 元素间运算
    // Vector RowVector 是特殊的Matrix
    Matrix2d a;
    a << 1, 2,
         3, 4;
    Matrix2d b;
    b << 5, 6,
         7, 8;
    cout << "a + b = " << endl << a+b << endl;
    cout << "a - b = " << endl << a-b << endl;
    cout << "Doing a += b. ";
    a += b;
    cout << "Now a = " << endl << a << endl;
    cout << "a * 2 = " << endl << a*2 << endl;
    cout << "a / 2 = " << endl << a/2 << endl;
}

void test08()
{   
    // 快速初始化
    MatrixXd m1 = MatrixXd::Random(3,3); // 随机从[-1,1]中生成
    MatrixXd m2 = MatrixXd::Constant(3,3,5); // 元素为常量5
    Matrix2d m3 = Matrix2d::Zero(); // 元素为0
    Matrix3d m4 = Matrix3d::Ones(); // 元素为1
    Matrix4d m5 = Matrix4d::Identity(); // 单位矩阵
    cout << "m1 = " << endl << m1 << endl;
    cout << "m2 = " << endl << m2 << endl;
    cout << "m3 = " << endl << m3 << endl;
    cout << "m4 = " << endl << m4 << endl;
    cout << "m5 = " << endl << m5 << endl;
    // 矩阵操作
    Matrix2cf m6 = Matrix2cf::Random();
    cout << "The transpose of m6 = " << endl << m6.transpose() << endl; // 转置
    cout << "The conjugate of m6 = " << endl << m6.conjugate() << endl; // 共轭
    cout << "The adjoint of m6 = " << endl << m6.adjoint() << endl; // 伴随 共轭转置
    MatrixXd m7(2,3); m7 << 1,2,3,4,5,6;
    cout << "Before being transpose, m7 = " << endl << m7 << endl;
    // m7 = m7.transpose() 不要这样操作
    m7.transposeInPlace(); // 这样操作
    cout << "After being transpose, m7 = " << endl << m7 << endl;
}

void test09()
{
    // 矩阵乘法
    Matrix2d m; 
    m << 1,2,
         3,4;
    Vector2d u(-1,1), g(2,0);
    cout << "m*m = " << endl << m*m << endl;
    cout << "m*u = " << endl << m*u << endl;
    cout << "u^T*m = " << endl << u.transpose()*m << endl;
    cout << "u^T*v = " << endl << u.transpose() * g << endl;
    cout << "u*v^T = " << endl << u * g.transpose() << endl;
    // 向量乘法
    Vector3d v(1,2,3);
    Vector3d w(0,1,2);
    cout << "Dot product: " << v.dot(w) << endl;
    double dp = v.adjoint()*w;
    cout << "Dot product via a matrix product: " << dp << endl;
    cout << "Cross product: " << endl << v.cross(w) << endl; 
}

void test10()
{
    // 矩阵元素操作
    Matrix2d mat;
    mat << 1, 2,
           3, 4;
    cout << "Here is mat.sum(): " << mat.sum() << endl; // 元素求和
    cout << "Here is mat.prod(): " << mat.prod() << endl; // 元素求积
    cout << "Here is mat.mean(): " << mat.mean() << endl; // 元素求平均
    cout << "Here is mat.minCoeff(): " << mat.minCoeff() << endl; // 最小元素
    cout << "Here is mat.maxCoeff(): " << mat.maxCoeff() << endl; // 最大元素
    cout << "Here is mat.trace(): " << mat.trace() << endl; // 迹 对角矩阵元素的和
    cout << "Here is the trace of mat: " << mat.diagonal().sum() << endl; // 

    Matrix3f m = Matrix3f::Random();
    ptrdiff_t i, j; // 最值对应的索引
    float minOfM = m.minCoeff(&i,&j); // 最值对应的索引
    cout << "Here is the matrix m:\n" << m << endl;
    cout << "Its minimum coefficient (" << minOfM 
         << ") is at position (" << i << "," << j << ")\n\n";
    
    RowVector4f v = RowVector4f::Random();
    ptrdiff_t k;
    float maxOfV = v.maxCoeff(&k);
    cout << "Here is the vector v: " << v << endl;
    cout << "Its maximum coefficient (" << maxOfV 
         << ") is at position " << k << endl;
}

void test11()
{
    // Array数据结构操作
    // 可以进行矩阵元素之间的运算 加减乘除同一个数 或者元素间加减乘除
    ArrayXXf a(3,3);
    a << 1,2,3,
         4,5,6,
         7,8,9;
    ArrayXXf b(3,3);
    b << 1,2,3,
         1,2,3,
         1,2,3;
    cout << "a + b = \n" << a+b << endl;
    cout << "a - b = \n" << a-b << endl;
    cout << "a * b = \n" << a*b << endl;
    cout << "a / b = \n" << a/b << endl;
    
    Array33f c = Array33f::Random();
    cout << "c = \n" << c << endl;
    cout << "c.abs() =\n" << c.abs() << endl;
    cout << "c.abs().sqrt() =\n" << c.abs().sqrt() << endl;
}

void test12()
{
    // Matrix 和 Array 之间的转换
    Matrix2d m1; m1 << 1,2,3,4;
    Matrix2d m2; m2 << 1,1,1,1;
    Array22d a1; a1 << 1,2,3,4;
    Array22d a2; a2 << 1,1,1,1;
    m1 = a1 * a2; // Array隐式转换成Matrix
    cout << "m1 = \n" << m1 << endl;
    a1 = m1 * m2; // Matrix隐式转换成Array
    cout << "a1 = \n" << a1 << endl;
    m2 = m1 + a1.matrix(); // 需要显式转换才能运算
    a2 = a1 + m1.array();
    cout << "m2 = \n" << m2 << endl;
    cout << "a2 = \n" << a2 << endl;
    // m1a是m1.array()的别名, 它们共享相同的系数
    ArrayWrapper<Matrix2d> m1a(m1);
    MatrixWrapper<Array22d> a1m(a1);
    cout << "a2 = \n" << a1 + m1a << endl;
}

void test13()
{
    // 矩阵块操作
    Matrix4f m(4, 4);
    for (int i = 0; i < 4; ++i) 
    {
        for (int j = 0; j < 4; ++j) 
        {
            m(i, j) = j + 1 + i * 4;
        }
    }
    cout << "m = \n" << m << endl;
    // 作为右值使用
    // (p,q)是块的大小 (i,j)是块的起始位置
    // matrix.block<p,q>(i,j) 固定尺寸块的表达
    cout << "block in the middle: " << endl;
    cout << m.block<2,2>(1,1) << endl << endl;
    // matrix.block(p,q,i,j) 动态尺寸块的表达
    for (int i = 1; i <=3; i++)
    {
        cout << "Block of size " << i << "x" << i << endl;
        cout << m.block(0,0,i,i) << endl << endl;
    }
    // 作为左值使用
    Matrix2f mat; mat << 1,2,3,4;
    Array44f arr = Array44f::Constant(0.5);
    cout << "Here is the array: \n" << arr << endl;
    arr.block<2,2>(1,1) = mat;
    cout << "After block operation: \n" << arr << endl;
    cout << "Here is the matrix: \n" << mat << endl;
    mat.block<1,2>(0,0) = mat.block<1,2>(1,0);
    cout << "After block operation: \n" << mat << endl;
    // row 和 col 操作
    cout << "The 1st row of matrix: \n" << mat.row(0) << endl;
    cout << "The 2nd col of matrix: \n" << mat.col(1) << endl;
    mat.col(1) += 3*mat.col(0);
    cout << "Here is the matrix: \n" << mat << endl;
    // 边角相关操作
    cout << "Here is the matrix: \n" << m.topLeftCorner(2,2) << endl; // 左上角 2x2
    cout << "Here is the matrix: \n" << m.bottomRightCorner(2,2) << endl; // 右下角 2x2
    cout << "Here is the matrix: \n" << m.topRows(1) << endl; // 上面前1行元素
    cout << "Here is the matrix: \n" << m.leftCols(2) << endl; // 左边前两行元素
    // 向量操作
    Vector4f vec(1,2,3,4);
    cout << "Here is the vector: \n" << vec.head(2) << endl; // 前2元素
    cout << "Here is the vector: \n" << vec.tail(2) << endl; // 后2元素
    cout << "Here is the vector: \n" << vec.segment(1,3) << endl; // 1到3
}

void test14()
{
    Vector3f v(5,6,7);
    cout << "v.norm() = " << v.norm() << endl; // 范数
    cout << "v.squareNorm() = " << v.squaredNorm() << endl; // 二范数
    cout << "v.lpNorm<Infinity>() = " << v.lpNorm<Infinity>() << endl; // 无穷范数
    cout << endl;
    Matrix2f m; m << 1,2,3,4;
    cout << "m.norm() = " << m.norm() << endl; // 范数
    cout << "m.squareNorm() = " << m.squaredNorm() << endl; // 二范数
    cout << "m.lpNorm<Infinity>() = " << m.lpNorm<Infinity>() << endl; // 无穷范数
}

void test15()
{
    // 布尔
    Array22f a; a << 1,2,3,4;
    cout << "(a > 0).all()   = " << (a > 0).all() << endl;
    cout << "(a > 0).any()   = " << (a > 0).any() << endl;
    cout << "(a > 0).count() = " << (a > 0).count() << endl;
    cout << endl;
    cout << "(a > 2).all()   = " << (a > 2).all() << endl;
    cout << "(a > 2).any()   = " << (a > 2).any() << endl;
    cout << "(a > 2).count() = " << (a > 2).count() << endl;
}

void test16()
{
    // 获取索引
    Matrix2f m; m << 1, 2, 3, 4;
    // 获取最大值索引
    MatrixXf::Index maxRow, maxCol;
    float max = m.maxCoeff(&maxRow, &maxCol);
    // 获取最小值索引
    MatrixXf::Index minRow, minCol;
    float min = m.minCoeff(&minRow, &minCol);
    cout << "Max: " << max <<  ", at: " <<
        maxRow << "," << maxCol << endl;
    cout << "Min: " << min << ", at: " <<
        minRow << "," << minCol << endl;
}

void test17()
{
    // 每行每列的最值
    MatrixXf mat(2,4);
    mat << 1, 2, 6, 9,
            3, 1, 7, 2;
    std::cout << "Column's maximum: " << std::endl
        << mat.colwise().maxCoeff() << std::endl;
    std::cout << "Row's maximum: " << std::endl
        << mat.rowwise().maxCoeff() << std::endl;
    // 元素和最大的一列
    MatrixXf::Index maxIndex;
    float maxNorm = mat.colwise().sum().maxCoeff(&maxIndex);
    std::cout << "Maximum sum at position " << maxIndex << std::endl;
    std::cout << "The corresponding vector is: " << std::endl;
    std::cout << mat.col( maxIndex ) << std::endl;
    std::cout << "And its sum is is: " << maxNorm << std::endl;
}

void test18()
{
    // 广播机制 
    MatrixXf mat(2,4);
    VectorXf v(2);
    mat << 1, 2, 6, 9,
            3, 1, 7, 2;
    v << 0,1;
    // 矩阵每一列都加上向量
    mat.colwise() += v;
    std::cout << "Broadcasting result: " << std::endl;
    std::cout << mat << std::endl;
}

void test19()
{
    Matrix3d rotation_matrix;
    rotation_matrix.setIdentity();
    // 旋转向量 由旋转轴和旋转角度组成
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout.precision(3);
    cout << "rotation vector: Angle is: " << rotation_vector.angle() * (180 / M_PI)
        << "  Axis is: " << rotation_vector.axis().transpose() << endl;
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 自由向量的旋转
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
    
    // 欧拉角 按ZYX的顺序 由旋转矩阵直接转换成欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "yaw pitch roll = " << euler_angles.transpose() * (180 / M_PI) << endl;
    
    // 齐次变换矩阵  4x4的
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    //    T.rotate(rotation_matrix);    // 这样写也行，相当于由旋转矩阵构造变换矩阵
    // 设置平移向量
    T.pretranslate(Vector3d(0, 0, 3)); // pre是左乘
    cout << "Transform matrix = \n" << T.matrix() << endl;
    cout << "Rotation = \n" << T.rotation() << endl;
    cout << "Translation = \n" << T.translation().transpose() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v;
    cout << "v transformed = " << v_transformed.transpose() << endl;

    // 由旋转向量构造四元数
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion = \n" << q.coeffs() << endl;
    // 由旋转矩阵构造四元数
    q = Quaterniond(rotation_matrix);
    cout << "quaternion = \n" << q.coeffs() << endl;
    v_rotated = q * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
}

void test20()
{
    // 旋转矩阵
    Matrix3d rotation_matrix;
    // 轴角
    rotation_matrix = AngleAxisd(M_PI / 6, Vector3d::UnitZ()) *
                    AngleAxisd(M_PI / 4, Vector3d::UnitY()) *
                    AngleAxisd(M_PI / 3, Vector3d::UnitX());
    cout << rotation_matrix << endl << endl;
    // 欧拉角
    Vector3d euler_angle = rotation_matrix.eulerAngles(2,1,0);
    cout << "yaw pitch roll : " << euler_angle.transpose() * (180/M_PI) << endl << endl;
    // 四元数
    Quaterniond quaternion(rotation_matrix);
    cout << quaternion.x() << " ";
    cout << quaternion.y() << " ";
    cout << quaternion.z() << " ";
    cout << quaternion.w() << endl; 
    // 齐次变换矩阵
    // 利用变换矩阵模板
    Isometry3d T1 = Isometry3d::Identity();
    T1.rotate(rotation_matrix);
    T1.pretranslate(Vector3d(0,0,0));
    cout << "Homogenous matrix: " << endl << T1.matrix() << endl << endl;;
    // 利用分块矩阵
    Matrix4d T2 = Matrix4d::Identity();
    T2.block<3,3>(0,0) = rotation_matrix;
    // T2.block<3,1>(0,3) = Vector3d(1,1,1);
    T2.topRightCorner(3,1) = Vector3d(1,1,1);
    cout << "Homogenous matrix: " << endl << T2 << endl;
}


int main()
{
    Vector3d a(-7,5,8), b(1,1,1);
    // cout << a.cwiseProduct(b) << endl;
    cout << a.cwiseMin(6).cwiseMax(-6).transpose() << endl;
    cout << a.transpose() << endl;
    
    return 0;
}
