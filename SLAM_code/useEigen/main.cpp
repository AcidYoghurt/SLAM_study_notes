#include <iostream>
#include <string>
using namespace std;
#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

// 本程序演示了 Eigen 几何模块的使用方法
int main(int argc, char **argv) {
   // Eigen/Geometry 模块提供了各种旋转和平移的表示

   // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
   Matrix3d rotation_matrix = Matrix3d::Identity();

   // 旋转向量使用 AngleAxisd
   AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
   cout.precision(3);
   cout << "旋转角 = " << rotation_vector.angle() << endl << "旋转轴 = " << rotation_vector.axis().transpose() << endl;
   cout << "旋转矩阵 =\n" << rotation_vector.matrix() << endl;   //用matrix()转换成矩阵

   // 旋转向量 转换为 旋转矩阵


   // 用 旋转向量 进行坐标变换
   Vector3d v(1, 0, 0);
   Vector3d v_rotated = rotation_vector * v;
   cout << "(1,0,0）旋转后（使用旋转向量） = " << v_rotated.transpose() << endl;
   // 用 旋转矩阵 进行坐标变换
   v_rotated = rotation_matrix * v;
   cout << "(1,0,0) 旋转后（使用旋转矩阵） = " << v_rotated.transpose() << endl;

   // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
   Vector3d euler_angles_1 = rotation_matrix.canonicalEulerAngles(0, 1, 2); // XYZ顺序，即roll-pitch-yaw顺序
   Vector3d euler_angles = rotation_matrix.canonicalEulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
   cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

   // 欧氏变换矩阵使用 Eigen::Isometry
   Isometry3d T = Isometry3d::Identity();                            // 虽然称为3d，实质上是4*4的矩阵（因为是齐次）
   T.rotate(rotation_vector);                                        // 定义旋转：按照rotation_vector进行旋转
   T.pretranslate(Vector3d(1, 3, 4));                          // 定义平移：把平移向量设成(1,3,4)
   cout << "变换矩阵 = \n" << T.matrix() << endl;

   // 用变换矩阵进行坐标变换
   Vector3d v_transformed = T * v;                                   // 相当于R*v+t
   cout << "v tranformed = " << v_transformed.transpose() << endl;

   // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

   // 四元数
   // 可以直接把AngleAxis赋值给四元数，反之亦然
   Quaterniond q = Quaterniond(rotation_vector);
   cout << "四元数（由旋转向量转换） = " << q.coeffs().transpose() << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
   // 也可以把旋转矩阵赋给它
   q = Quaterniond(rotation_matrix);
   cout << "四元数（由旋转矩阵转换） = " << q.coeffs().transpose() << endl;
   // 使用四元数旋转一个向量，使用重载的乘法即可
   v_rotated = q * v; // 注意 数学上是qvq^{-1}，但是在写代码的时候Eigen库帮我们简化了，Eigen库内部执行的是(q * Quaternion(0,v) * q.inverse()).vec()
   cout << "(1,0,0)使用四元数旋转后 = " << v_rotated.transpose() << endl;
   // 用常规向量乘法表示，则应该如下计算
   Quaterniond qvq = q * Quaterniond(0, 1, 0, 0) * q.inverse();
   cout << "q*v跟q * Quaterniond(0, 1, 0, 0) * q.inverse()的值是相等的 = " << qvq.coeffs().transpose() << endl;

   return 0;
}
