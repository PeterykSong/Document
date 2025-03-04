[[053_Neuralangelo High-Fidelity Neural Surface Reconstruction.pdf]]

# 1. 요약

논문은 고해상도 3D 표면 재구성을 수행하는 Neuralangelo라는 프레임워크를 제안합니다. 기존의 뉴럴 표면 재구성(neural surface reconstruction) 방법들은 세부 구조를 복원하는 데 한계를 가지지만, Neuralangelo는 **다중 해상도 해시 그리드(multi-resolution hash grids)** 와 **뉴럴 표면 렌더링(neural surface rendering)** 기법을 결합하여 고품질의 3D 모델을 생성합니다.

Neuralangelo는 다음의 두 가지 핵심 기술을 활용합니다:

1. **수치적 그래디언트(numerical gradients)**: 고차 도함수를 계산하여 표면의 매끄러움을 유지하고 최적화를 안정화함.
2. **점진적(coarse-to-fine) 최적화**: 해시 그리드의 해상도를 점진적으로 증가시키면서 세부 구조를 복원하는 방법.

이 방법은 깊이(depth) 등의 보조 데이터 없이 RGB 영상만으로 고밀도의 3D 표면을 재구성할 수 있으며, 기존 방법보다 훨씬 높은 충실도를 보장합니다. 실험 결과, Neuralangelo는 DTU 데이터셋과 Tanks and Temples 데이터셋에서 이전 방법들보다 더 높은 정확도를 보이며, 3D 장면을 더 세밀하게 복원할 수 있음을 증명하였습니다.

# 2. 용어

- **뉴럴 표면 재구성(Neural Surface Reconstruction)**
    - 뉴럴 네트워크를 활용하여 다중 시점의 RGB 이미지로부터 3D 표면을 복원하는 기술.
    - 기존의 멀티뷰 스테레오(MVS) 기법보다 더 세밀한 구조를 복원할 수 있음.
    
- **뉴럴 볼륨 렌더링(Neural Volume Rendering)**
    - 뉴럴 네트워크를 활용하여 3D 공간의 밀도와 색상을 학습하고 이를 기반으로 새로운 시점에서 이미지를 합성하는 기술.
    - 대표적인 기법으로 **NeRF (Neural Radiance Fields)** 가 있음.
-	**SDF, Signed Distance Function**
    -  3D 공간의 각 점에서 표면까지의 부호가 있는 거리 값을 나타내는 함수.
    -  뉴럴 네트워크를 활용하여 장면을 암시적(implicit)으로 표현하는 데 사용됨.
    
- **다중 해상도 해시 그리드(Multi-Resolution Hash Grid)**
    - 공간 좌표를 해시 인코딩하여 저장하고, MLP와 결합하여 3D 구조를 효율적으로 학습하는 기법.
    - **Instant NGP (Neural Graphics Primitives)** 를 기반으로 함.
    
- **수치적 그래디언트(Numerical Gradients)**
    - 표면의 법선(surface normal)과 같은 고차 도함수를 더 안정적으로 계산하기 위해 사용.
    - 기존의 해시 그리드 기반 분석적 그래디언트보다 더 부드러운 표면을 생성하는 데 기여함.
    
- **점진적 최적화(Coarse-to-Fine Optimization)**
    - 초기에는 거친(low-resolution) 해상도로 학습을 시작하고 점진적으로 세부 해상도를 증가시켜 최적화하는 기법.
    - 최적화가 빠른 지역 최소값(local minima)에 빠지지 않도록 유도함.
    
- **Chamfer Distance**
    - 3D 재구성의 정확도를 평가하는 지표로, 두 3D 점 집합(point cloud) 사이의 평균 거리를 측정.
    
- **PSNR (Peak Signal-to-Noise Ratio)**
    - 이미지 품질을 평가하는 지표로, 원본 이미지와 생성된 이미지의 차이를 수치적으로 평가.
    
# 3. 주요 내용

#### 1) 뉴럴 볼륨 렌더링 (Neural Volume Rendering)

#### 2) **SDF 기반 볼륨 렌더링 (Volume Rendering of SDF)**

#### 3) 다중 해상도 해시 인코딩 (Multi-Resolution Hash Encoding)

#### 4) 수치적 그래디언트 계산 (Numerical Gradient Computation)

#### 5) 점진적 수준 최적화 (Progressive Levels of Details)

#### 6) 최적화 (Optimization)

#### 7)**곡률 정규화 (Curvature Regularization)**

#### 8) **위상 워밍업 (Topology Warmup)**


# 4. 실험

# 5. 고찰
