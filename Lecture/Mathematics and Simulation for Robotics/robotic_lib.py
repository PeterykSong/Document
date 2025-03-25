from roboticstoolbox import DHRobot, RevoluteDH
import roboticstoolbox as rtb
import spatialmath as sm
import matplotlib.pyplot as plt

# DH 파라미터 정의 (a, alpha, d, theta offset)
# 예: 3자유도 로봇팔
robot = DHRobot([
    RevoluteDH(a=1, alpha=0,    d=0, offset=0),
    RevoluteDH(a=1, alpha=0,    d=0, offset=0),
    RevoluteDH(a=0.5, alpha=0,  d=0, offset=0)
], name="3DOF_Arm")

print(robot)  # 로봇 구조 출력