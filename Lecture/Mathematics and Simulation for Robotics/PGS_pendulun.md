## ✅ Double Pendulum의 Constrained Dynamics 운동 방정식 (LaTeX 표현)

### 1️⃣ 상태 변수 정의

두 링크의 끝점 좌표:
$$
q = \begin{bmatrix} x_1 \\ y_1 \\ x_2 \\ y_2 \end{bmatrix}
$$

### 2️⃣ 질량 행렬 (Mass matrix)
$$
M = \begin{bmatrix}
m_1 & 0 & 0 & 0 \\
0 & m_1 & 0 & 0 \\
0 & 0 & m_2 & 0 \\
0 & 0 & 0 & m_2
\end{bmatrix}
$$

### 3️⃣ 외력 (External forces)
$$
F_{ext} = \begin{bmatrix} 0 \\ -m_1 g \\ 0 \\ -m_2 g \end{bmatrix}
$$

### 4️⃣ 제약 조건 (Constraints)
첫 번째 끝점은 원 위:
$$
C_1(q) = x_1^2 + y_1^2 - l_1^2 = 0
$$
두 번째 끝점은 첫 번째 끝점으로부터 거리 $l_2$ 유지:
$$
C_2(q) = (x_2 - x_1)^2 + (y_2 - y_1)^2 - l_2^2 = 0
$$

### 5️⃣ 제약 조건 Jacobian (J)
$$
J = \begin{bmatrix}
2x_1 & 2y_1 & 0 & 0 \\
-2(x_2 - x_1) & -2(y_2 - y_1) & 2(x_2 - x_1) & 2(y_2 - y_1)
\end{bmatrix}
$$

### 6️⃣ 운동 방정식 (Constrained form)
$$
M \ddot{q} + J^T \lambda = F_{ext}
$$
$$
C(q) = 0
$$
여기서 $\lambda$는 라그랑주 승수이며, constraint force를 의미합니다.

### 7️⃣ Time-stepping (이산화 형태)
$$
M (v_{k+1} - v_k) = \Delta t (F_{ext} + J^T \lambda)
$$
$$
q_{k+1} = q_k + \Delta t \cdot v_{k+1}
$$

### 8️⃣ 요약
- 운동 방정식과 제약 조건을 함께 풀어 $\lambda$ (제약력)를 구하고
- 속도 및 위치를 time-stepping 방식으로 계산합니다.
