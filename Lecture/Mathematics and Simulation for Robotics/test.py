from sympy import pi

# Let's build a function that returns the homogeneous transform from base to each joint frame
def compute_fk_from_dh(dh_params):
    T_list = []
    T_total = eye(4)
    for row in dh_params:
        theta, d, a, alpha = row
        T = dh_transform(theta, d, a, alpha)
        T_total = simplify(T_total * T)
        T_list.append(T_total)
    return T_list

# Build the DH table using symbolic variables
theta1, theta2, theta3, theta5, theta6 = symbols('theta1 theta2 theta3 theta5 theta6')
d4 = symbols('d4')

dh_table = [
    (theta1, 0, 1, 0),
    (theta2, 0, 1, pi/2),
    (theta3, 1, 0, 0),
    (pi/2, d4, 0, pi/2),
    (theta5, 0, 1, 45 * deg),
    (theta6, 1, 0, -135 * deg)
]

T_frames = compute_fk_from_dh(dh_table)

# We'll show transformation matrices for each frame wrt base
T_frame_dicts = [{f"T0_{i+1}": T} for i, T in enumerate(T_frames)]
T_frame_dicts
