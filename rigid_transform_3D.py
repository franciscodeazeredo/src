#!/usr/bin/python
import numpy as np
# Input: expects 3xN matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B
    return R, t

def create_numpy_array(points):
    # Extract coordinates from the list of points and create a NumPy array
    coordinates = [[point.x, point.y, point.z] for point in points]
    return np.array(coordinates)

def continuous_rigid_transform(point_list_1, point_list_2):
    # Create NumPy arrays from the lists of points
    A = create_numpy_array(point_list_1)
    B = create_numpy_array(point_list_2)

    # Find the rigid transformation
    R, t = rigid_transform_3D(A, B)

    # Find the new coordinates of the points in point_list_1
    A2 = (R @ A) + t

    # Find the difference between the coordinates of the points in point_list_2
    # and the coordinates of the points in point_list_1 after the rigid transformation
    diff = A2 - B

    # Find the sum of the squared differences
    sum_squared_diff = np.sum(diff * diff)

    # Find the root mean squared error
    rmse = np.sqrt(sum_squared_diff / len(point_list_1))
    print(f"RMSE: {rmse}")
    return R, t, rmse

def transform_points(points, R, t):
    # Create NumPy array from the list of points
    A = create_numpy_array(points)

    # Find the new coordinates of the points
    A2 = (R @ A) + t

    # Create a list of Point objects
    transformed_points = [Point(x, y, z) for x, y, z in A2.T]
    return transformed_points
