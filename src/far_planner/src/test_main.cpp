#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cassert>
#include <iostream>
#include <unordered_map>

#include "far_planner/point_struct.h"
using namespace std;
int main(int argc, char** argv) {
    std::cout << "--------开始测试----------" << std::endl;
    Point3D p1(1.0f, 2.0f, 3.0f);
    Point3D p2(2.0f, 3.0f, 4.0f);
    Point3D p3 = p1 + p2;
    std::cout << "p1: " << p1 << std::endl;  // 测试 operator<<
    std::cout << "p3 (p1+p2): " << p3 << std::endl;
    assert(p3.x == 3.0f && "p3.x 应该等于 3.0");  // 测试 operator+

    Point3D p4 = p1 * 3.0f;
    std::cout << "p4 (p1*3): " << p4 << std::endl;
    assert(p4.y == 6.0f && "p4.y 应该等于 6.0");  // 测试 operator*

    // 2. 测试成员函数
    Point3D p_norm(3.0f, 4.0f, 0.0f);
    std::cout << "p_norm的2D模长: " << p_norm.norm_flat() << std::endl;
    assert(fabs(p_norm.norm_flat() - 5.0f) < 1e-6 && "2D模长计算错误");

    Point3D p_unit = p_norm.normalize_flat();
    std::cout << "p_unit (归一化): " << p_unit << std::endl;
    assert(fabs(p_unit.x - 0.6f) < 1e-6 && "归一化错误");

    // 3. 测试与外部库的桥梁 (PCL 和 Eigen)
    PCLPoint pcl_p;
    pcl_p.x = 5;
    pcl_p.y = 6;
    pcl_p.z = 7;
    Point3D p_from_pcl(pcl_p);
    std::cout << "从PCL点转换: " << p_from_pcl << std::endl;
    assert(p_from_pcl.x == 5 && "PCL 构造失败");

    Eigen::Vector3d eig_v(8, 9, 10);
    Point3D p_from_eigen(eig_v);
    std::cout << "从Eigen向量转换: " << p_from_eigen << std::endl;
    assert(p_from_eigen.y == 9 && "Eigen 构造失败");

    // 4. 测试哈希 (高级)
    // 如果这能编译通过，你的 point_hash 就基本写对了
    std::unordered_map<Point3D, std::string, point_hash> test_map;
    test_map[p1] = "点1";
    test_map[p_from_pcl] = "PCL点";

    std::cout << "哈希表测试: p1 对应的值是 " << test_map[p1] << std::endl;
    assert(test_map[p1] == "点1" && "哈希表实现失败");

    std::cout << "--- 所有测试通过 ---" << std::endl;
    return 0;
}