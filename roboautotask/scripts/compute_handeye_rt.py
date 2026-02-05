'''
使用SVD空间坐标系转换算法计算R, t
手动修改输入文件
'''
import numpy as np

def main():
    # 读取文件（请将 'points.txt' 替换为你的文件路径）
    data = np.loadtxt('cabrition.txt')
    
    if data.shape[1] != 6:
        raise ValueError("每行必须包含6个数字：x y z X Y Z")
    
    robot_points = data[:, :3]   # 目标点（Robot 空间）
    cam_points = data[:, 3:]     # 源点（Cam 空间）

    # 计算质心
    centroid_cam = np.mean(cam_points, axis=0)
    centroid_robot = np.mean(robot_points, axis=0)

    # 去中心化
    cam_centered = cam_points - centroid_cam
    robot_centered = robot_points - centroid_robot

    # 协方差矩阵
    H = cam_centered.T @ robot_centered

    # SVD
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T

    # 计算旋转矩阵
    R = V @ U.T

    # 处理反射（确保右手系）
    if np.linalg.det(R) < 0:
        V[:, -1] *= -1
        R = V @ U.T

    # 计算平移
    t = centroid_robot - R @ centroid_cam

    # 输出结果
    print("Rotation matrix R:")
    print(R)
    print("\nTranslation vector t:")
    print(t)

if __name__ == '__main__':
    main()
    