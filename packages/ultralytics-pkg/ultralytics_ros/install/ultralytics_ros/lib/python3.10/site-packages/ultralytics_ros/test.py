import numpy as np

def estimate_affine_transform():
    source_points = np.array([
        (1.87, 1.06),
        (2.20, 0.48),
        (0.87, 0.35),
        (1.12, 1.027),
        (0.19, 0.42),
        (0.19, 1.32),
        (0.85, 1.72)
        
    ])

    target_points = np.array([
        (1.895, 0.95),
        (2.22, 0.25),
        (0.78, 0.25),
        (1.095, 0.95),
        (0.075, 0.395),
        (0.075, 1.32),
        (0.82, 1.725)
    ])

    # 增加一列 1，用來做仿射轉換計算
    N = len(source_points)
    A = np.hstack([source_points, np.ones((N, 1))])  # Nx3
    B = target_points  # Nx2

    # 求解最小平方差解
    X, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

    # 分離仿射轉換矩陣與平移量
    affine_matrix = X[:2].T  # 2x2 矩陣
    offset = X[2]            # 平移量 b

    return affine_matrix, offset

# 測試
A, b = estimate_affine_transform()
print("Affine matrix A:\n", A)
print("Translation b:\n", b)
x = 2.2
y = 0.48
pt = np.array([x, y])
corrected = A @ pt + b
print ("Corrected point:\n", corrected)