
|     방식      |  연도  |  차원   | Map 저장 방식   | 기타  |
| :---------: | :--: | :---: | ----------- | --- |
|  FastSLAM   |      |  2D   |             |     |
|    LOAM     | 2014 |  3D   | Point Cloud |     |
|    iSAM     | 2008 |       |             |     |
|   LIO SAM   | 2020 | 2D+3D |             |     |
| ScanContext | 2021 |  2D   | Voxel+SCD   |     |
|  PIN SLAM   | 2024 | 2D+3D | Voxel+SDF   |     |
|  FAST-LOI2  | 2021 |  3D   | Point?      |     |


# Particle filter (Fast SLAM)


# Graph SLAM
[[Document/Paper Review/신입생세미나/043_graph-SLAM.pdf|043_graph-SLAM]]


# LOAM 
Zhang, Ji & Singh, Sanjiv. (2014). LOAM: Lidar Odometry and Mapping in Real-time. 10.15607/RSS.2014.X.007. 
[[Document/Paper Review/신입생세미나/023_LOAM.pdf|023_LOAM]]
### Key Point


### Algorithm


### 장점


### 단점


### 후속연구



# Lio SAM
Shan, Tixiao, et al. "Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping." _2020 IEEE/RSJ international conference on intelligent robots and systems (IROS)_. IEEE, 2020.
[[021_LIO SLAM.pdf]]
### Key Point


### Algorithm


### 장점


### 단점


### 후속연구



# iSAM



# PIN SLAM


# GMapping


# Cartograper


# FAST LIO2
 - 문제의식
	 - Building a dense 3-dimension (3D) map of an unknown environment in real-time and simultaneously localizing in the map (i.e., SLAM) is crucial for autonomous robots 
	 - The central requirement for adopting LiDAR-based SLAM approaches to these widespread applications is to obtain accurate, low-latency state estimation and dense 3D map with limited onboard computation resource
	 - the performance of the feature extraction module is easily influenced by the environment.
	    if the LiDAR Field of View (FoV) is small, a typical phenomenon of emerging solid-state LiDARs [16].   
	 -  LiDAR points are usually sampled sequentially while the sensor undergoes continuous motion. This procedure creates significant motion distortion influencing the perfor- mance of the odometry and mapping, especially when the motion is severe
	 -  4) LiDAR usually has a long measuring range (e.g., hundreds of meters) but with quite low resolution between scanning lines in a scan. The resultant point cloud measurements are sparsely distributed in a large 3D space, necessitating a large and dense map to register these sparse points. 


- 제안/접근법
	- incremental k-d tree (ikd tree) and direct points registration.

- 참고연구들
	- LiDAR SLAM들
		- Lego-LOAM : Ground Point Segmentation
		- LOAM Livox : LOAM + SolidState LiDAR
		- LION [28] : Loosely coupled IMU and LiDAR
		- LILIOM [17] 
		- LIO-SAM [30] requires a 9-axis IMU to produce attitude measurement as the prior of scan registration within a small local map
		- LINS [31] introduces a tightly- coupled iterated Kalman filter and robocentric formula into  the LiDAR pose optimization in the odometry.
		- FAST-LIO [22] introduces a formal back-propagation that precisely considers the sampling time of every single point in a scan and compensates the motion distortion via a rigorous kinematic model driven by IMU measurement
		    - Kalman gain formula is used to reduce the computation complexity from the dimension of the measurements to the dimension of the state.
	- Mapping
		- kNN search problem can be solved by building spatial indices for data points, which can be divided into two categories: partitioning the data and splitting the space.
		-  R-tree [37] which clusters the data into potential overlapped axis-aligned cuboids based on data proximity in space
		- R∗-tree which outperforms the original ones [38].
		- Octree [39] and k-dimensional tree (k-d tree) [40] are two well-known types of data structures to split the space for kNN search. 
		- Mapping methods using k-d tree libraries, such as ANN [44], libnabo [43] and FLANN [45], fully re-build the k-d trees to update the map, which results in considerable computation. 



https://www.youtube.com/watch?v=M-GWxY2L_Fs
https://gsk1m.github.io/

# 3월 목표 1: lidar slam history 정리


-----------------------------

#### 01. '87 PAMI Least-squares fitting of two 3D point sets  
Least-squares Fitting of Two 3-D Point Sets" 1987년 IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI, Vol. 9, No. 5, pp. 698-700)
	-  두 개의 3D 점 집합 $\{p_i\}\{p_i'\}$ 와 (여기서 i=1,2,...,Ni = 1, 2, ..., N`i = 1, 2, ..., N`)가 주어졌을 때, 이들이 다음과 같은 관계로 연결된다고 가정합니다:
	- $$p'_i=Rp_i+T+N_i$$
	-  여기서:
		   - ( R ): 회전 행렬 (Rotation Matrix)
		   - ( T ): 이동 벡터 (Translation Vector)
		   - $N_i$  :  노이즈 벡터 (Noise Vector)
	-  목표는 주어진 점 집합 $\{p_i\}\{p_i'\}$ 를 이용해 ( R )과 ( T )의 최소 제곱 해를 찾는 것.
	- 3×3 행렬의 특이값 분해(Singular Value Decomposition, SVD)를 기반으로 알고리즘을 제시
	- 알고리즘 개요
		1. 중심화(Centering): 두 점 집합의 중심(평균 위치)을 계산하고, 각 점을 중심화하여 이동 성분을 제거합니다.
		2. 상관 행렬 계산: 두 중심화된 점 집합 간의 상관 행렬 ( H )를 계산합니다.
		3. SVD 적용: ( H ) 행렬에 대해 특이값 분해를 수행하여 $H=UΣV^T$  를 얻습니다.
		4. 회전 행렬 추정: $R=VU^T$    로 회전 행렬을 구합니다. (단, ( R )이 올바른 회전 행렬이 되도록 보정 필요)
		5. 이동 벡터 계산: ( R )을 이용해 $T=p′_c − Rp_c$   를 계산합니다. 
		   (여기서 $p_c$ , $p_c`$  는 각각 점 집합의 중심)


--------------------------------------

#### 02. '92 PAMI ICP  
Paul J. Besl, Neil D. McKay, A Method for Registration of 3-D Shapes" (IEEE Transactions on Pattern Analysis and Machine Intelligence, Vol. 14, No. 2, February 1992, pp. 239-256)
	
	- 신입생 세미나때 발표한 논문
	- Iterative 과정을 가져왔기 때문에 위의 87년 논문대비 개선점이 있음. ICP의 시작
	

----------------------------

#### 03. '97 AR Globally consistent range scan alignment  
Lu, F., & Milios, E. (1997). Globally consistent range scan alignment for environment mapping. Autonomous Robots, 4(2), 333-349. [https://doi.org/10.1023/A:1008817304089](https://doi.org/10.1023/A:1008817304089)
- 배경 및 문제 정의
	- 로봇이 환경을 탐색하며 수집한 센서 데이터(특히 범위 스캔)를 통합하여 일관된 세계 모델을 생성하려면, 각 데이터 프레임을 정확히 정합하는 것이 필수적입니다. 기존의 점진적 접근법(incremental approach)에서는 새로운 데이터 프레임을 기존의 글로벌 모델에 정합한 뒤 병합하는 방식이 사용되었습니다. 그러나 이 방식은 정합 과정에서 발생하는 오류로 인해 모델의 서로 다른 부분이 독립적으로 갱신되면서 일관성이 깨질 수 있다는 단점이 있었습니다.
- 연구 목적
		- 이 논문은 여러 범위 스캔 프레임의 일관된 정합 문제를 연구하며, 공간적 불확실성(spatial uncertainties)의 표현과 조작 문제를 함께 다룹니다. 목표는 모든 로컬 데이터 프레임과 그 사이의 상대적 공간 관계를 유지하면서 글로벌 일관성을 확보하는 방법론을 제안하는 것입니다.
- 제안된 접근법
		- 로컬 프레임과 관계 유지: 모든 로컬 데이터 프레임을 유지하고, 프레임 간 상대적 공간 관계를 네트워크 형태로 관리합니다. 이 관계는 쌍별 스캔 매칭(pairwise scan matching) 또는 오도메트리(odometry)로부터 도출되며, 랜덤 변수로 모델링됩니다.
		- 최적 포즈 추정: 최대 우도 기준(maximum likelihood criterion)을 기반으로 모든 공간 관계를 최적으로 결합하는 절차를 제안합니다. 이를 통해 데이터 프레임의 포즈(pose)를 동시에 해결함으로써 일관성을 달성합니다.
		- 오차 최소화: 쌍별 스캔 매칭에서 발생하는 포즈 관계를 Mahalanobis 거리 형태로 표현하고, 이를 최소화하는 방식으로 최적화합니다.
- 방법론의 핵심
		1. 네트워크 구축: 로봇의 포즈를 노드로, 스캔 간 겹침(overlap)을 기반으로 링크를 정의합니다.
		2. 공간 관계 도출: 오도메트리나 스캔 매칭을 통해 포즈 간 제약 조건을 생성합니다.
		3. 최적화: 목적 함수를 설정하고 이를 최소화하여 모든 포즈를 동시에 조정합니다.
- 실험 
		- 논문에서는 시뮬레이션 데이터와 실제 데이터를 사용한 실험 결과를 제시합니다. 이를 통해 제안된 방법이 기존 점진적 접근법보다 일관된 환경 매핑을 제공함을 입증했습니다.

---------------------------

#### 04. '03 IROS NDT registration  ***
Magnusson, M., Duckett, T., & Johansson, A. (2003). Scan registration for autonomous mining vehicles using 3D-NDT. In Proceedings of the 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 856-861). IEEE. [https://doi.org/10.1109/IROS.2003.1248818](https://doi.org/10.1109/IROS.2003.1248818)

- 배경 및 목적
	- 이 논문은 자율 주행 광산 차량이 복잡하고 비정형적인 광산 환경에서 정확한 위치 추정과 맵핑을 수행할 수 있도록, 3D 스캔 데이터를 정합(registration)하는 새로운 방법을 제안합니다. 당시 널리 사용되던 ICP(Iterative Closest Point) 알고리즘은 점 대 점 매칭에 의존하여 계산 비용이 높고 초기 정렬에 민감한 단점이 있었습니다. 이에 비해 NDT(Normal Distributions Transform)는 점군(point cloud)을 정규 분포의 집합으로 변환하여 더 효율적이고 견고한 정합을 가능하게 합니다.
- 방법론: 3D-NDT
	- NDT는 스캔 데이터를 일정한 크기의 셀(voxel)로 나누고, 각 셀 내 점들의 분포를 정규 분포(평균과 공분산)로 모델링합니다. 이 접근법은 개별 점의 정확한 매칭 대신 분포 간의 유사성을 최적화하여 정합을 수행합니다. 주요 단계는 다음과 같습니다:
		1. 데이터 표현: 입력된 3D 점군을 그리드 셀로 분할하고, 각 셀의 점들을 정규 분포로 변환.
		2. 목표 함수 정의: 두 스캔 간의 정합 품질을 측정하기 위해, 변환된 점군과 참조 스캔 간의 분포 유사성을 최대화하는 목표 함수를 설정.
		3. 최적화: 뉴턴-랩슨(Newton-Raphson)과 같은 비선형 최적화 기법을 사용해 변환 파라미터(회전, 이동)를 계산.
		4. 반복: 초기 정렬을 바탕으로 점진적으로 정합을 개선.
	 - 이 방법은 연속적인 공간 표현을 제공하므로, ICP처럼 초기 추정값에 크게 의존하지 않고도 안정적인 결과를 도출할 수 있습니다.
 - 실험 환경
	 - 저자들은 실제 광산 환경에서 수집된 3D 레이저 스캔 데이터를 사용해 알고리즘을 테스트했습니다. 광산은 구조적 특징이 부족하고 먼지, 불규칙한 표면 등으로 인해 스캔 정합이 어려운 환경으로 알려져 있습니다. 실험은 자율 차량에 장착된 3D 스캐너로 수집된 데이터를 기반으로 진행되었으며, NDT와 ICP의 성능을 비교했습니다.
 - 결과
	 - 효율성: 3D-NDT는 ICP에 비해 계산 시간이 적게 소요되었으며, 특히 대규모 점군 데이터에서 더 빠른 수렴을 보였습니다.
	 - 정확성: 광산 환경의 노이즈와 불완전한 데이터에도 불구하고, NDT는 일관된 정합 결과를 제공하며 더 높은 정확도를 달성했습니다.
	 - 견고성: 초기 정렬 오류에 덜 민감하여, 초기 추정값이 부정확하더라도 성공적으로 스캔을 정합할 수 있었습니다.
	 - 응용 가능성: 이 기술은 실시간 위치 추정과 맵핑에 적합하며, 자율 내비게이션 시스템에 통합될 잠재력을 입증했습니다.
 - 이후 연구
	 - "The Three-Dimensional Normal-Distributions Transform – An Efficient Representation for Registration, Surface Analysis, and Loop Detection" (2007)
		 - 이 논문은 3D-NDT를 스캔 정합뿐만 아니라 표면 분석과 루프 클로저 검출에 활용할 수 있는 다목적 도구로 확장합니다. Magnusson은 NDT를 이용해 점군 데이터를 효율적으로 압축하고, 이를 로봇의 위치 추정 및 맵핑에 적용하는 방법을 탐구했습니다. 실험은 실내 및 실외 환경에서 수행되었으며, NDT가 기존 방법보다 메모리 사용량이 적고 계산 속도가 빠르다는 점을 강조합니다
	 - "Evaluation of 3D Registration Reliability and Speed – A Comparison of ICP and NDT"(2009)
		 - 이 논문은 3D-NDT와 ICP의 성능을 체계적으로 비교하며, 정합의 신뢰도와 속도를 평가합니다. 다양한 데이터셋(실내, 실외, 광산 환경)을 사용해 실험한 결과, NDT가 초기 정렬에 덜 의존하고 더 빠른 수렴 속도를 보였습니다. 또한, NDT의 매개변수 튜닝(예: 셀 크기)을 최적화하여 성능을 개선한 방법을 제안합니다.

-----------------------

#### 05. '06 IJRR Square Root SAM

Dellaert, F., & Kaess, M. (2006). Square Root SAM: Simultaneous Localization and Mapping via Square Root Information Smoothing. International Journal of Robotics Research, 25(12), 1181–1204. [https://doi.org/10.1177/0278364906072768](https://doi.org/10.1177/0278364906072768)

- LAM(Simultaneous Localization and Mapping) 문제를 해결하기 위한 새로운 접근법인 "Square Root SAM(√SAM)"을 제안합니다. SLAM은 로봇이 미지의 환경에서 자신의 위치를 추정하고 동시에 지도를 생성하는 문제로, 전통적으로 확장 칼만 필터(EKF)를 기반으로 한 방법이 주로 사용되었습니다. 그러나 이 논문에서는 EKF 대신 평활화(smoothing) 접근법을 사용하여 정보 행렬 또는 측정 야코비 행렬을 제곱근 형태로 분해하는 기법을 탐구합니다.

- SAM의 주요 장점은 다음과 같습니다
	- 속도와 정확성: EKF보다 빠르면서도 정확한 결과를 제공합니다.
	- 유연성: 배치(batch) 모드와 증분(incremental) 모드 모두에서 사용할 수 있습니다.
	- 비선형 모델 처리: 비선형 프로세스 및 측정 모델을 더 잘 다룰 수 있습니다.
	- 전체 궤적 제공: 로봇의 전체 이동 궤적을 더 낮은 비용으로 얻을 수 있습니다.
    
	또한, 이 방법은 SLAM 문제의 지리적 특성에서 비롯된 지역성을 자동으로 활용하며, 희소 선형 대수학을 기반으로 최적화 문제를 효율적으로 해결합니다. 논문은 이러한 방법의 이론적 기반을 제시하고, 그래픽 모델 관점에서 분해를 해석하며, 시뮬레이션 결과와 실제 SLAM 실험을 통해 √SAM이 EKF 기반 접근법에 비해 잠재력을 가진 대안임을 입증합니다.

	이 논문은 로봇 공학 분야에서 SLAM 알고리즘의 성능을 개선하고자 하는 연구자들에게 중요한 통찰을 제공하며, 이후 iSAM과 같은 증분 평활화 방법으로 발전하는 기초를 마련했습니다.
 1. 문제 배경
    	당시 주류였던 확장 칼만 필터(EKF) 기반 SLAM은 다음과 같은 한계가 있었습니다:
    	- 계산 복잡도: 변수가 증가할수록 계산량이 기하급수적으로 늘어남.
    	- 오차 누적: 비선형 모델에서 오차가 누적되어 정확도가 떨어질 수 있음.
    	- 단일 상태 추정: 현재 상태만 추정하며 과거 데이터를 활용하기 어려움.
    -    이에 반해, Dellaert와 Kaess는 "평활화(smoothing)" 접근법을 통해 전체 궤적을 추정하고 계산 효율성을 높이는 방법을 제안합니다.

2. Square Root SAM(√SAM)의 핵심 아이디어
   √SAM은 정보 행렬(Information Matrix) 또는 측정 야코비 행렬을 제곱근 형태로 분해하여 SLAM 문제를 해결합니다. 이를 통해 다음과 같은 목표를 달성합니다:
	- 효율성: 희소(sparse) 행렬 연산을 활용해 계산 비용을 줄임.
	- 정확성: 비선형 최적화를 통해 전체 데이터를 일관되게 처리.
	- 전체 궤적 추정: 과거와 현재 데이터를 모두 활용해 로봇의 전체 경로와 지도를 최적화.
	
	 2.1 평활화(Smoothing) vs 필터링(Filtering)
	 - 필터링(EKF): 매 시간 단계마다 상태를 업데이트하며 과거 데이터를 버림.
	 - 평활화(√SAM): 전체 시간 동안의 측정값을 한꺼번에 처리해 최적의 궤적을 계산. 이는 그래프 최적화 문제로 변환됩니다.
	 2.2 그래프 기반 접근
		√SAM은 SLAM을 **팩터 그래프(Factor Graph)** 로 모델링합니다:
		- 노드(Node): 로봇 위치(poses)와 랜드마크(landmarks).
		- 엣지(Edge): 위치 간 이동(odometry)이나 랜드마크 관측(measurements).
		- 이 그래프를 통해 측정값과 상태 변수 간의 관계를 표현하고, 비선형 최소 제곱 문제로 최적화합니다.
    2.3 제곱근 분해(Square Root Decomposition)
		- 기존 정보 행렬 $H^TH$  (정규 방정식의 형태)를 직접 계산하는 대신, ( H )를 QR 분해하거나 Cholesky 분해를 통해 제곱근 형태 ( R )로 변환합니다:
		-  $H^T H = R^T R$
		- 여기서 ( R )은 상삼각 행렬(upper triangular matrix)로, 희소성과 지역성을 활용해 계산이 효율적입니다.
3. 수학적 기반
	1. SLAM 문제를 수학적으로 표현하면, 상태 벡터 $X = \{x_1, x_2, ..., x_n, l_1, l_2, ..., l_m\}$ 
	   (로봇 위치 ( x )와 랜드마크 ( l ))와 측정값 $Z = \{z_1, z_2, ..., z_k\}$ 가 주어졌을 때, 조건부 확률 ( P(X|Z) )를 최대화하는 ( X )를 찾는 것이 목표입니다. 이를 로그 우도 함수로 변환하면:

		$\log P(X|Z) = -\frac{1}{2} \sum || h_i(X) - z_i ||^2_{\Sigma_i}$
		여기서 $h_i(X)$ 는 측정 모델이고,$\Sigma_i$ 는 측정 노이즈 공분산입니다. 이 식을 최소화하는 문제는 비선형 최소 제곱 문제로, 야코비 행렬 ( H )를 사용해 선형화되고, ( R ) 행렬로 변환됩니다.

		√SAM은 정보 행렬 $H^T H$ 를 직접 계산하는 대신, 제곱근 형태 ( R )로 분해합니다:
		$H^T H = R^T R$  
		여기서 ( R )은 상삼각 행렬(upper triangular matrix)로, 희소성과 지역성을 활용해 계산이 효율적입니다.


---------------

#### 06. '08 TRO iSAM  *** 
Kaess, M., Ranganathan, A., & Dellaert, F. (2008). iSAM: Incremental Smoothing and Mapping. IEEE Transactions on Robotics, 24(6), 1365–1378. [https://doi.org/10.1109/TRO.2008.2006706](https://doi.org/10.1109/TRO.2008.2006706)
- 개요

이 논문은 로봇 공학에서 SLAM 문제를 해결하기 위한 새로운 접근법인 **iSAM (Incremental Smoothing and Mapping)** 을 제안합니다. SLAM은 로봇이 미지의 환경에서 자신의 위치를 추정하고 동시에 환경의 지도를 작성하는 문제로, 계산 복잡성과 실시간 처리 요구로 인해 어려움이 많습니다. 기존의 방법들은 주로 필터링 기반 (예: EKF, Particle Filter) 또는 배치 최적화 (Batch Optimization)에 의존했으나, iSAM은 **증분적 매끄럽게 하기 (Incremental Smoothing)** 를 통해 효율적이고 정확한 해결책을 제공합니다.

핵심 기여

1. 증분적 행렬 분해:  
    iSAM은 SLAM의 정보 행렬 (Information Matrix)을 QR 분해를 통해 증분적으로 업데이트합니다. 이를 통해 전체 행렬을 매번 다시 계산하지 않고, 변화가 발생한 부분만 효율적으로 갱신합니다. 이는 특히 긴 경로를 따라 이동하거나 루프 클로저 (Loop Closure)가 많은 경우에도 계산 효율성을 유지합니다.
    
2. 희소성 활용:  
    SLAM 문제에서 자연스럽게 발생하는 희소 행렬 (Sparse Matrix)의 특성을 활용하여 불필요한 계산을 줄이고 메모리 사용을 최적화합니다. 주기적인 변수 재정렬 (Variable Reordering)을 통해 행렬의 Fill-in을 최소화합니다.
    
3. 실시간 데이터 연관:  
    실시간으로 데이터 연관 (Data Association)을 수행할 수 있도록, 추정 불확실성 (Estimation Uncertainty)에 빠르게 접근하는 알고리즘을 제공합니다. 이는 로봇이 실시간으로 환경을 탐색하며 지도를 갱신할 때 유용합니다.
    

방법론

- 문제 정의: SLAM은 로봇의 위치와 랜드마크의 위치를 동시에 추정하는 비선형 최적화 문제로 모델링됩니다. iSAM은 이를 확률적 관점에서 접근하며, 모든 관측값을 활용해 전체 후방 분포 (Full Posterior)를 추정합니다.
    
- 알고리즘 구조:
    
    1. 새로운 관측 데이터가 들어올 때마다 기존의 정보 행렬을 증분적으로 갱신.
        
    2. 주기적으로 변수 순서를 재정렬하여 희소성을 유지.
        
    3. 필요 시 추정값과 불확실성을 계산하기 위해 역행렬 연산 대신 효율적인 백서브스티튜션 (Back-substitution)을 사용.
        
- 구현: iSAM은 랜드마크 기반 SLAM과 자세만 추정하는 SLAM (Pose-only SLAM) 모두에 적용 가능하도록 설계되었습니다.
    

실험 결과

저자들은 다양한 시뮬레이션 데이터셋과 실제 데이터셋 (예: Victoria Park 데이터)을 사용해 iSAM의 성능을 평가했습니다. 결과는 다음과 같습니다:

- 효율성: 기존의 배치 방식 대비 계산 시간이 크게 단축됨.
    
- 정확성: EKF 기반 방법보다 더 정확한 지도와 위치 추정 제공.
    
- 확장성: 루프 클로저가 많은 복잡한 환경에서도 안정적으로 동작.
    

한계 및 향후 연구

- iSAM은 데이터 연관이 이미 해결되었다고 가정하며, 불확실한 데이터 연관 문제는 다루지 않음.
    
- 비선형성이 강한 경우 주기적인 재선형화 (Relinearization)가 필요할 수 있음. 이후 연구에서 이를 개선한 iSAM2가 제안됨 (2012년).
    

결론

iSAM은 SLAM 문제를 실시간으로 해결할 수 있는 강력한 도구로, 증분적 업데이트와 희소 행렬 활용을 통해 효율성과 정확성을 동시에 달성했습니다. 이 논문은 이후 iSAM2 및 관련 연구의 기반을 마련하며, 로봇 내비게이션과 매핑 분야에서 중요한 기여를 했습니다.

--------------------------------------------

#### 07. '09 ICRA Olson (correlative scan matching)  
E. B. Olson, "Real-time correlative scan matching," in Proceedings of the 2009 IEEE International Conference on Robotics and Automation (ICRA), Kobe, Japan, May 12-17, 2009, pp. 4387-4393, doi: 10.1109/RO еслиBOT.2009.5152375.

1. 배경 및 문제 정의
	 -  문제: 두 개의 2D 레이저 스캔   $S_1$ (참조 스캔)과  $S_2$ (새 스캔)가 주어졌을 때, $S_2$ 를 $S_1$     에 정합(align)시키는 변환 $T = (x, y, \theta)$  (x, y 이동과 회전 각도)를 찾는 것. 
	 - 목표 함수: $S_1$ 과 $T$ 로 변환된 $S_2$ 간의 일치도를 최대화.
	 - 기존 한계: ICP는 초기값에 의존적이고, 상관 기반 방법은 계산 비용이 높아 실시간 적용이 어려움.  

2. 제안된 방법: Correlative Scan Matching
	 2.1 핵심 아이디어
	 - 상관 기반 접근법: 가능한 모든 변환 ( T )에 대해  $S_1$  과 $T(S_2)$ 의 상관 값을 계산하고, 이를 최대화하는  $T^*$ 를 찾음.
    
	- 수식: 상관 값 ( C(T) )는 다음과 같이 정의:
		$C(T) = \sum_{i} M(S_1, T(S_2)_i)$ 
	    여기서 ( M )은 $S_1$ 의 occupancy grid 표현이고, $T(S_2)_i$    는 변환된 $S_2$ 의 각 포인트.
    
	2.2 알고리즘 구조
		1. Occupancy Grid 생성:   
		    - $S_1$ 을 이산화된 2D그리드 (M)으로 변환. 각 셀은 점유 확률(occupied/free)을 나타냄.   
		    - 해상도 예: 5cm × 5cm 셀.
		2. 변환 공간 이산화:     
		    - $T = (x, y, \theta)$  의 검색 공간을 그리드로 나눔.
		    - 예:     $\Delta x = \Delta y = 5 \, \text{cm}, \Delta \theta = 1^\circ$ 
		3. 상관 계산: 
			- $S_2$  의 각 포인트를 ( T )로 변환 후 ( M )에서 점유 값을 조회.       
		    - 점유 값의 합이 상관 스코어 ( C(T) )가 됨.
		4. 멀티레벨 탐색:
			  -  낮은 해상도: 큰 간격(예: $\Delta x = 20 \, \text{cm}, \Delta \theta = 4^\circ$ )으로 대략적 최적 ( T ) 탐색. 
			  - 높은 해상도: 초기 결과 주변에서 세밀한 간격으로 정밀화.
			  - 속도 향상을 위해 피라미드 구조 사용.
	2.3 확률 모델
		- 센서 노이즈를 고려해 점유 값을 확률적으로 계산:
			-  $$P(\text{occupied} | S_1, T(S_2)) = \frac{1}{Z} \exp\left(-\frac{d^2}{2\sigma^2}\right)$$ 
				- ( d ): $S_2$  포인트와  $S_1$ 의 최근접 점 거리. 
				- $\sigma$  :  센서 불확실성 파라미터. 
				- ( Z ): 정규화 상수. 
		- 이는 단순한 이진 점유(0/1) 대신 연속적인 확률 값을 제공해 견고함 강화.
	2.4 최적화
	- 브루트 포스 계산 비용을 줄이기 위해:
		- FFT(Fast Fourier Transform): 평행 이동(x, y)에 대해 상관 계산을 가속화.
		- 분기 한정(Branch-and-Bound): 비유망 변환을 조기에 배제.
        

3. 기술적 세부사항

	- 입력: 레이저 스캔 데이터 (일반적으로 180° 또는 360° 범위, 수백 개 포인트).
	- 출력: 최적 변환   $T^* = (x^*, y^*, \theta^*)$  
	- 복잡도:     
	    - 단일 해상도: $O(N \cdot X \cdot Y \cdot \Theta)$ , 여기서 ( N )은 포인트 수, $X, Y, \Theta$ 는 변환 공간 크기.
	    - 멀티레벨: 실질적 복잡도 감소 (예: $O(N \cdot \log(X \cdot Y \cdot \Theta))$ )
	- 구현: C++로 작성, 단일 코어 2GHz CPU에서 실시간 동작 확인.

4. 실험 결과
	- 환경: Intel Core 2 Duo, 2GHz, 2009년 데이터셋 (MIT Killian Court 등).
    - 성과:
        - 속도: 평균 1020ms (50100Hz).
	    - 정확도: 평균 오차 $< 2 \, \text{cm}, < 0.5^\circ$ 
	- 비교:
        - ICP: 초기값 오류 시 실패율 높음.
        - NDT: 계산 비용 높음 (50~100ms).
	- 시각화: 논문 내 그림에서 스캔 정합 전/후 결과 제공.    
5. 한계
	- 해상도 의존성: 그리드 간격이 너무 크면 국부 최적해에 갇힐 수 있음.
	- 회전 범위:  $\theta$  탐색 범위가 제한적일 경우 큰 회전 추정 실패 가능성.    
	- 계산 부하: 포인트 수가 많거나 검색 공간이 클 때 여전히 부담.

--------------------------
#### 08.  '09 ICRA FPFH registration  

R. B. Rusu, N. Blodow, and M. Beetz, "Fast Point Feature Histograms (FPFH) for 3D registration," in *Proc. 2009 IEEE International Conference on Robotics and Automation (ICRA)*, Kobe, Japan, May 2009, pp. 3212-3217, doi: 10.1109/ROBOT.2009.5152473.

"Fast Point Feature Histograms (FPFH) for 3D Registration" 논문은 3D 포인트 클라우드 데이터를 정합(registration)하기 위한 효율적인 특징 기술자(descriptor)를 제안합니다. 이 논문에서 소개된 **FPFH (Fast Point Feature Histograms)** 는 기존의 **PFH (Point Feature Histograms)** 를 개선하여 계산 효율성을 높이고, 실시간 응용 프로그램에 적합하도록 설계되었습니다. 

- 배경
 3D 포인트 클라우드 정합은 로보틱스, 컴퓨터 비전, 3D 재구성 등에서 중요한 문제입니다. 기존 PFH는 포인트 주변의 기하학적 정보를 히스토그램으로 표현하여 강력한 특징을 제공했지만, 계산 복잡도가 높아 실시간 처리에 제약이 있었습니다. FPFH는 이를 해결하기 위해 계산량을 줄이면서도 PFH의 강력한 특성을 유지하려는 시도입니다.

- 주요 기여
	1. 계산 효율성 개선: FPFH는 PFH의 복잡도를  $O(nk^2)$  에서 $O(nk)$로 줄였습니다. 
	   여기서 (n)은 포인트 수, (k)는 이웃 포인트 수입니다.    
	2. 실시간 적용 가능성: 효율적인 알고리즘으로 로봇 내비게이션이나 물체 인식 같은 실시간 응용에 활용 가능.
	3. 정합 성능 유지: 단순화된 계산에도 불구하고, FPFH는 PFH와 비슷한 수준의 정합 정확도를 제공.
    
- 방법론
	- FPFH는 각 포인트의 기하학적 특징을 계산할 때, 직접적인 이웃뿐만 아니라 간접적인 이웃 관계를 간소화된 방식으로 반영합니다. 이를 통해 특징 기술자의 품질을 유지하면서도 처리 속도를 높였습니다.

- 실험 결과
	논문에서는 다양한 3D 데이터셋(스캔된 물체, 실내 환경 등)을 사용해 FPFH의 성능을 평가했습니다. 결과적으로 FPFH는 PFH 대비 속도가 10배 이상 빠르면서도 정합 오류율은 비슷하거나 약간 낮은 수준을 보였습니다.

- 결론
	FPFH는 계산 효율성과 정합 성능의 균형을 맞춘 기술자로, 실시간 3D 처리에 적합한 솔루션을 제공합니다.
	
 - 기술적인 상세사항
	1. PFH와의 차이점
		- PFH: 각 포인트 (p)에 대해 (k)-최근접 이웃((k)-NN)을 찾고, 모든 이웃 쌍(pairwise) 간의 기하학적 관계(법선 벡터 사이의 각도 등)를 계산합니다. 이 과정에서 $k^2$  개의 계산이 필요해 복잡도가 높습니다. 
		- FPFH: (p)와 각 이웃 간의 관계만 직접 계산하고, 이웃 간 상호작용은 간접적으로 반영합니다. 이를 통해 복잡도를($O(k)$ 로 줄임.
    2. FPFH 계산 과정
		FPFH는 두 단계로 나뉩니다:
		1. SPFH (Simplified Point Feature Histograms) 계산
			- 포인트 (p)와 (k)-최근접 이웃 $p_i$  간의 기하학적 특징을 계산.        
			- 세가지 각도(법선 벡터와 방향 벡터 간의 관계)를 기반으로 히스토그램 생성: 
			    - $\alpha = \mathbf{v} \cdot \mathbf{n}_t$ : 법선 간 각도.           
		        - $\phi = (\mathbf{u} \cdot (p_t - p_s))/d$  : 방향 벡터와 거리 투영. 
		        - $\theta = \arctan(\mathbf{w} \cdot \mathbf{n}_t, \mathbf{u} \cdot \mathbf{n}_t)$ : 법선과 방향 간 각도.
		        - 여기서 $\mathbf{u}, \mathbf{v}, \mathbf{w}$ 는 로컬 좌표계(Darboux frame)를 정의하며, (d)는 두 점 간 유클리드 거리입니다.
		    - 이 값들을 히스토그램으로 변환해 SPFH를 생성.
		2. FPFH 확장 
			- (p)의 SPFH와 이웃 $p_i$ 의 SPFH를 가중 평균으로 결합:
                $$FPFH(p) = SPFH(p) + \frac{1}{k} \sum_{i=1}^{k} \frac{1}{w_i} \cdot SPFH(p_i)$$
			- $w_i$ 는 (p)와 $p_i$ 간 거리로, 가까운 이웃일수록 더 큰 영향을 미침.
		3. 최적화
			- 캐싱: 반복 계산되는 법선 벡터와 거리 값을 미리 저장해 중복 연산을 줄임. 
			- k-d 트리: 이웃 검색을 효율화하기 위해 k-d 트리 구조 사용.

		4. 성능 분석
			- 시간 복잡도: PFH는 $O(nk^2)$  인 반면, FPFH는 $O(nk)$로 단일 포인트당 계산량이 대폭 감소.
			- 메모리 사용량: 히스토그램 크기는 여전히 고정(기본 설정에서 33개의 bin 사용)으로, 메모리 효율도 유지.
		5. 응용
			FPFH는 이후 PCL(Point Cloud Library)에 구현되어 3D 정합, 물체 인식, SLAM(Simultaneous Localization and Mapping) 등에 널리 사용됩니다.

---------------------
#### 09. '09 RSS GICP  

A. Segal, D. Hähnel, and S. Thrun, "Generalized-ICP," in Proc. Robotics: Science and Systems (RSS), 2009.

"Generalized-ICP"는 3D 점군(Point Cloud) 데이터를 정합(Alignment)하는 데 사용되는 알고리즘으로, 기존의 ICP(Iterative Closest Point) 방법론을 개선한 연구입니다. 점군 정합은 로봇공학이나 컴퓨터 비전에서 중요한 문제로, 예를 들어 로봇이 센서로 수집한 두 개의 3D 지도를 하나로 합치거나 물체의 위치를 추정할 때 필요합니다.

기존 ICP는 두 점군 간의 가장 가까운 점들을 찾아 반복적으로 정합을 수행하지만, 잡음(Noise)이나 불완전한 데이터에 취약하다는 단점이 있었습니다. GICP는 이를 보완하기 위해 점 간 거리뿐만 아니라 점들의 "표면 특성"(예: 곡률, 방향)을 고려하고, 확률적 모델(Probability Model)을 도입해 더 정확하고 견고한 결과를 제공합니다. 논문은 실험을 통해 GICP가 기존 ICP보다 오류를 줄이고 성능을 향상시켰음을 입증했습니다.

1. 기본 ICP의 한계
	기존 ICP는 두 점군 $P = \{p_i\}$ (소스)와 $Q = \{q_j\}$  (타겟)을 정합을 위해 다음단계를 반복합니다:
		- 각 $p_i$  에 대해 ( Q )에서 가장 가까운 점 $q_j$  를 찾음 (Correspondence).
		- 두 점 쌍 간의 거리 제곱합을 최소화하는 변환(회전 ( R ), 이동 ( T ))을 계산: 		  $$\text{argmin}_{R,T} \sum_i \| p_i' - q_j \|^2, \quad p_i' = R p_i + T$$
		  - 변환을 적용하고 수렴할 때까지 반복.   
	하지만 이 방식은:
	- 점 간 단순 유클리드 거리만 고려하므로 표면 구조를 무시.
	- 잡음, 이상치(Outlier), 또는 점군 밀도 차이에 취약.
    - 국부 최적해(Local Minimum)에 빠질 가능성 높음.
    
2. GICP의 확률적 접근
	GICP는 점들을 단순한 좌표가 아닌, 확률 분포로 모델링합니다. 각 점 $p_i$ 와 $q_j$ 는 센서 잡음을 반영한 가우시안 분포로 표현됩니다:
		- $p_i \sim \mathcal{N}(\mu_{p_i}, C_{p_i})$ : 소스 점의 평균과 공분산.
		- $q_j \sim \mathcal{N}(\mu_{q_j}, C_{q_j})$ : 타겟 점의 평균과 공분산.
	
	여기서 공분산 ( C )는 점 주변의 로컬 표면 특성을 반영합니다. 예를 들어:
		 - 평평한 표면에서는 공분산이 표면 법선 방향으로 작고, 평면에 수직한 방향으로 큼.   
		- 곡률이 높은 모서리에서는 공분산이 더 균일.
		
3. 목적 함수 재정의
	GICP는 점 간 거리 대신, 두 가우시안 분포 간의 차이를 최소화하는 목적 함수를 사용합니다. 두 점 $p_i$ 와 $q_j$ 간 잔차는:
			  
			  $$d_i = p_i' - q_j$$ 
			  
	이 잔차의 "비용"은 공분산을 고려한 마할라노비스 거리(Mahalanobis Distance)로 계산됩니다:
	 
	 $$\text{Cost} = \sum_i d_i^T (C_{p_i} + C_{q_j})^{-1} d_i$$
	 
	 이를 최소화하는 ( R )과 ( T )를 찾는 것이 목표입니다. 여기서 $C_{p_i} + C_{q_j}$ 
	 는 두 분포의 결합 공분산으로, 표면 특성과 잡음을 동시에 반영합니다.
	 
4. 공분산 계산
   공분산 행렬 ( C )는 점군의 로컬 기하학을 기반으로 추정됩니다:
	- 각 점 주변의 이웃 점들을 분석해 주성분 분석(PCA)을 수행. 
	- 결과로 얻은 고유값(Eigenvalue)과 고유벡터(Eigenvector)를 사용해 공분산 행렬을 구성.
	- 예: 평면 표면의 경우, 법선 방향의 고유값은 작고, 평면 내 방향의 고유값은 큼.

5. 알고리즘 흐름
	1. 초기 변환 ( (R, T) )를 설정 (예: 단위 행렬).
    2. 소스 점군 ( P )를 변환: $P' = R P + T$    
    3. ( P' )와 ( Q ) 간 대응점(Correspondence)을 찾음 (최근접 점 탐색, kd-tree 활용).
    4. 공분산 기반 비용 함수를 최소화하도록 ( (R, T) )를 업데이트 (비선형 최적화, Levenberg-Marquardt 등 사용 가능).
    5. 수렴할 때까지 반복.
    
6. 성과 및 실험 결과
   논문은 GICP를 여러 데이터셋(예: LiDAR 스캔)에서 테스트하며, 기존 ICP 및 변형(예: Point-to-Plane ICP)과 비교했습니다:
	- 정확도: GICP는 평균 정합 오류를 약 20-30% 감소.
    - 견고성: 잡음과 이상치에 덜 민감.
    - 속도: 계산 복잡도는 ICP와 유사하나, 공분산 계산으로 약간의 오버헤드 발생.

----------------------------

#### 10. '10 ITSM A Tutorial on Graph-Based SLAM  ***

G. Grisetti, R. Kümmerle, C. Stachniss, and W. Burgard, "A tutorial on graph-based SLAM," IEEE Intell. Transp. Syst. Mag., vol. 2, no. 4, pp. 31–43, Winter 2010, doi: 10.1109/MITS.2010.939925.

"A Tutorial on Graph-Based SLAM"은 그래프 기반 SLAM(Simultaneous Localization and Mapping)에 대한 입문서로, 모바일 로봇이 알 수 없는 환경에서 자신의 위치를 추정하고 맵을 생성하는 문제를 다룹니다. 이 논문은 복잡한 수학적 개념을 비교적 이해하기 쉽게 설명하며, SLAM 문제를 그래프 최적화 문제로 변환하여 해결하는 방법을 제시합니다. 주요 내용은 다음과 같습니다:

- SLAM 개요:  
    SLAM은 로봇이 센서 데이터를 활용해 위치와 맵을 동시에 추정하는 과정입니다. 그래프 기반 접근법은 이 문제를 효율적으로 해결하기 위한 현대적인 방법론으로 자리 잡았습니다.
    
- 그래프 기반 표현:    
    - 로봇의 위치(또는 자세, pose)는 그래프의 노드로 표현됩니다.        
    - 센서 데이터(예: 이동 거리, 랜드마크 관측)에서 얻어진 제약 조건은 노드 간 엣지(edge)로 나타냅니다.
- 최적화:  
    그래프에 쌓인 오차를 최소화하기 위해 비선형 최소 제곱 최적화(non-linear least squares optimization)를 사용합니다. 이는 위치와 맵을 점진적으로 정제합니다.
    
- 구현:  
    실질적인 알고리즘과 오픈소스 라이브러리(G2O, TORO 등)를 소개하며, 이를 실제 로봇 시스템에 적용하는 방법을 설명합니다.

1. 문제 정의와 그래프 표현
   SLAM 문제는 로봇의 궤적 $X = \{x_1, x_2, ..., x_T\}$ (위치 또는 자세)와 맵 (M)을 추정하는 것으로, 다음과 같은 확률 분포를 최대화하는 과정입니다
		$$p(X, M | Z, U)$$
		    여기서:
		    - $Z = \{z_1, z_2, ..., z_T\}$  : 센서 관측값 (예: 레이저 스캔, 카메라 데이터)
		    - $U = \{u_1, u_2, ..., u_{T-1}\}$ : 제어 입력 (예: 오도메트리 데이터).    
	
	그래프 기반 SLAM에서는:
		- 노드: 각 시점 ( t )에서의 로봇 자세 $x_t$ 또는 랜드마크 위치.
		- 엣지: 두 노드 간의 공간적 제약 조건
		  (예: 오도메트리로 계산한 상대적 이동, 관측된 랜드마크와의 거리).

2. 비선형 최소 제곱 최적화
   그래프의 엣지는 오차 함수(error function)로 정의되며, 전체 오차는 다음과 같이 계산됩니다:
		 $$E(X) = \displaystyle\sum_{(i,j) \in C} { e_{ij}(x_i, x_j)^T \Omega_{ij} e_{ij}(x_i, x_j)}$$
		 -  $e_{ij}(x_i, x_j)$  : 노드 $x_i$ 와 $x_j$ 간 예측값과 실제 관측값의 차이.
	     - $\Omega_{ij}$ : 정보 행렬(information matrix), 관측값의 불확실성을 반영.    
		 - ( C ): 그래프의 엣지 집합.
    
    목표는 ( E(X) )를 최소화하는$X^*$ 를 찾는 것입니다:
		$X^* = \arg\min_X E(X)$
	이를 위해 Gauss-Newton 또는 Levenberg-Marquardt 같은 비선형 최적화 기법이 사용됩니다.

3. 알고리즘 흐름

	1.  그래프 구성:    
	    - 센서 데이터(오도메트리, 랜드마크 관측)를 기반으로 노드와 엣지를 추가.
		- 루프 폐쇄(loop closure): 로봇이 이전에 방문한 위치를 인식하면 새로운 엣지를 추가해 그래프를 업데이트.
        
	2. 최적화:
        - 초기 추정값(예: 오도메트리 기반)을 시작으로 반복적(iterative)으로 오차를 줄임.
        - 대규모 그래프의 경우, sparse matrix 연산(희소 행렬)을 활용해 계산 효율성을 높임.
        
	3. 결과: 최적화된 로봇 궤적과 맵.
    
4. 구체적인 기술적 예시
   논문에서는 G2O(General Graph Optimization) 프레임워크를 예로 듭니다:

	- 입력: 노드(로봇 자세, 랜드마크)와 엣지(제약 조건) 데이터.
	- 내부적으로 Cholesky 분해 또는 Schur 보완(Schur complement)을 사용해 대규모 시스템을 효율적으로 해결.    
	- 출력: 최적화된 좌표 집합.    

5. 실제 적용 시 고려사항
	- 센서 노이즈: 센서 데이터의 불확실성을 모델링하기 위해 공분산 행렬을 사용.
	- 루프 폐쇄 탐지: 잘못된 루프 폐쇄는 전체 그래프를 왜곡할 수 있으므로, 강건한 탐지 메커니즘이 필요.
	- 계산 비용: 그래프 크기가 커질수록 메모리와 연산 자원이 급격히 증가.    

6. 장점과 한계

- 장점: 확장 가능하고, 루프 폐쇄를 효과적으로 처리하며, 오차를 전역적으로 분산.
- 한계: 초기 추정값에 민감하고, 비선형성으로 인해 수렴이 보장되지 않을 수 있음.

-----------------------------------

#### 11. '12 RAM PCL tutorial  

R. B. Rusu and S. Cousins, "Point Cloud Library: A Modular Open-Source Library for Processing Point Clouds," IEEE Robotics and Automation Magazine, vol. 19, no. 3, pp. 45-55, Sept. 2012.

1. 서론
	- 배경: 3D 센서(예: LiDAR, Kinect)의 보급으로 포인트 클라우드 데이터가 로보틱스, 컴퓨터 비전, 자율 주행 등에서 중요해짐. 그러나 이를 처리하기 위한 통합적이고 모듈화된 도구가 부족했음.   
	- 목적: Point Cloud Library(PCL)를 소개하며, 3D 포인트 클라우드 데이터를 효율적으로 처리할 수 있는 오픈소스 라이브러리의 설계와 활용 방법을 설명.
	- 발표 시점: 2012년을 가정하므로, PCL의 초기 개발 단계에서 모듈화와 실용성을 강조.
   
2. PCL의 주요 특징

	- 모듈화 구조: PCL은 독립적인 모듈로 구성되어 있어 사용자가 필요한 기능만 선택 가능.
        - 예: 필터링, 특징 추출, 표면 재구성, 객체 인식 등.
	- 오픈소스: 누구나 무료로 사용 및 수정 가능하며, 커뮤니티 주도로 발전.
    - 호환성: C++ 기반으로 ROS(Robot Operating System)와 통합 가능.
    
3. 핵심 기능
	- 데이터 처리 파이프라인:
        1. 포인트 클라우드 입력: 3D 센서에서 생성된 데이터를 PCL 형식으로 변환.
	    2. 필터링: 잡음 제거 및 다운샘플링(예: Voxel Grid 필터).
		3. 특징 추출: 법선(Normal) 계산, 키포인트(Keypoint) 검출.
		4. 등록(Registration): 여러 포인트 클라우드를 정합(예: ICP 알고리즘).
		5. 표면 재구성: 포인트 클라우드에서 매끄러운 표면 생성(예: Marching Cubes).
	- 알고리즘 예시: PCL에서 제공하는 RANSAC, KD-Tree 등의 구현과 활용법 설명.
4. 튜토리얼 요소
	- 코드 예제: 간단한 포인트 클라우드 필터링 코드 제공.
	    
    cpp
	```cpp
    #include <pcl/point_cloud.h>
    #include <pcl/filters/voxel_grid.h>
    int main() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);
        return 0;
    }
    ```
    - 설치 가이드: PCL의 컴파일 및 환경 설정 방법 간략히 소개.
    - 활용 사례: 로봇 내비게이션, 3D 모델링 등에서의 실질적 적용 예시.
    

5. 성능 평가
	- 실험 결과: PCL의 알고리즘 속도와 정확도를 다른 라이브러리와 비교(예: OpenNI, MATLAB).
	- 결론: PCL은 속도와 유연성 면에서 우수하며, 모듈화로 확장 가능성 높음.
    

-----------------------------------
#### 12. '13 ICRA DVO (lidar 는 아니지만)  
C. Kerl, J. Sturm, and D. Cremers, "Dense Visual SLAM for RGB-D Cameras," in 2013 IEEE International Conference on Robotics and Automation (ICRA), Karlsruhe, Germany, May 2013, pp. 2100-2106, doi: 10.1109/ICRA.2013.6630850.

  - 이건 DSO 항목 참조할 것. 동일 저자 논문임. 
------------------------

#### 13. '13 AR Libpointmatcher  

Pomerleau, F., Colas, F., Siegwart, R., & Magnenat, S. (2013). Comparing ICP variants on real-world data sets. *Autonomous Robots*, 34(3), 133–148. https://doi.org/10.1007/s10514-012-9318-2

1. 연구 목적 및 배경

ICP 알고리즘은 두 포인트 클라우드(Point Cloud) 간의 정합(alignment)을 수행하는 대표적인 방법으로, 로보틱스와 컴퓨터 비전에서 널리 사용됩니다. 그러나 ICP는 초기 조건, 노이즈, 이상치(outlier) 등에 민감하며, 다양한 변형이 제안되어 왔습니다. 이 논문은 이러한 변형들을 체계적으로 비교하고, Libpointmatcher라는 오픈소스 라이브러리를 통해 재현 가능한 평가를 제공하는 것을 목표로 합니다.

- 핵심 질문: 어떤 ICP 변형이 특정 데이터셋에서 더 나은 성능을 보이는가?    
- 기여: (1) ICP 변형의 모듈화된 설계, (2) 실제 데이터셋 기반 비교, (3) Libpointmatcher 공개.
    

2. ICP 알고리즘 개요

ICP는 두 포인트 클라우드 ( P ) (고정)와 ( Q ) (이동)을 정합하기 위해 변환 행렬 ( T ) (회전 ( R )과 이동 ( t ))를 추정합니다. 기본 프로세스는 다음과 같습니다:

1. 대응점 매칭: ( Q )의 각 점에 대해 ( P )에서 가장 가까운 점을 찾음.    
2. 오차 최소화: 대응점 쌍 간 오차를 최소화하는 ( T )를 계산.    
3. 변환 적용: ( Q )에 ( T )를 적용하여 반복.    

수식으로 표현하면, 오차 함수는 다음과 같습니다:

$$E(R, t) = \displaystyle\sum_{i=1}^{N} \| p_i - (R q_i + t) \|^2$$ 

여기서:
	-  $p_i \in P$  : 고정 포인트 클라우드의 점
	- $q_i \in Q$   : 이동 포인트 클라우드의 점
	- ( R ): 3x3 회전 행렬 
	- ( t ): 3차원 이동 벡터
    

3. ICP 변형 분류

논문은 ICP를 5가지 주요 모듈로 분해하고, 각 모듈에서 변형을 분석합니다:

(1) 데이터 필터링 (Data Filters)

- 목적: 노이즈와 이상치를 줄여 입력 데이터를 전처리.    
- 기술:    
    - Random Sampling: 점을 무작위로 선택해 계산량 감소.        
    - Voxel Grid Filtering: 공간을 격자로 나누어 점 밀도 조정        
- 효과: 계산 속도 향상 및 이상치 제거.    

(2) 대응점 매칭 (Point Matching)

- 기술:    
    - Nearest Neighbor (NN): 가장 가까운 점 매칭.        
    - KD-Tree 기반 NN: 효율적 검색.        
    - Normal-based Matching: 점의 법선 벡터를 활용한 매칭.        
- 수식: NN 매칭은 유클리드 거리 기준:    
    $$c_i = \arg\min_{p_j \in P} \| p_j - q_i \|$$
	여기서 $c_i$  는 $q_i$ 에 대응하는 ( P )의 점.
    
(3) 오차 최소화 (Error Minimization)

- 기술:    
    - Point-to-Point: 두 점 간 유클리드 거리 최소화 (위의 ( E(R, t) )).
	- Point-to-Plane: 점과 평면 간 수직 거리 최소화.
         $$E(R, t) = \sum_{i=1}^{N} [(p_i - (R q_i + t)) \cdot n_i]^2$$
        여기서  $n_i$ 는 $p_i$ 의 법선 벡터.
        
	- 특징: Point-to-Plane은 평면 구조에서 더 빠른 수렴.
    

(4) 이상치 처리 (Outlier Rejection)

- 기술:
	- Trimmed ICP: 일정 비율의 대응점을 제외.
    - Distance Threshold: 거리가 임계값 이상인 쌍 제거.
	- 수식: Trimmed ICP는 ( E(R, t) )를 상위 ( k )개의 오차에 대해 계산.
    

(5) 수렴 조건 (Convergence Criteria)

- 기술:
	- 최대 반복 횟수.
    - 오차 변화율 임계값: $\Delta E < \epsilon$ 

4. 실험 설계

- 데이터셋:
	   - 실내(ETH Hauptgebäude), 실외(Planétarium), 구조적 환경 등 6개 실제 데이터셋.
    - 2D와 3D LiDAR 데이터 포함.        
	- 평가 지표:
	    - 정확도: 변환 후 RMSE(Root Mean Square Error).
		- 수렴 속도: 반복 횟수.
		- 계산 시간.

5. 주요 결과
	- Point-to-Plane vs Point-to-Point:
	    - Point-to-Plane은 구조적 환경에서 더 빠르게 수렴하며 RMSE 낮음.
	    - Point-to-Point는 노이즈가 많은 환경에서 견고함.
	- KD-Tree: 매칭 속도 개선에 기여.
	- Trimmed ICP: 이상치가 많은 데이터셋에서 성능 우수.
	- 데이터 필터링: Random Sampling은 속도를 높이나 정확도 손실 가능성 있음.
6. Libpointmatcher의 설계
	Libpointmatcher는 모듈화된 ICP 구현을 제공하며, 사용자가 각 모듈을 커스터마이징할 수 있도록 설계되었습니다:
	- 구성 요소: 데이터 필터, 매칭 함수, 오차 최소화기, 이상치 필터 등.
    - 기술적 특징: C++ 기반, ROS 통합 가능, KD-Tree 활용.


#### 14. 14 RSS LOAM (feature matching + scan to map)  
   신입생 세미나로 리뷰했음. 
#### 15. 15 ICRA VLOAM (visual + LOAM)   

J. Zhang and S. Singh, "Visual-lidar odometry and mapping: Low-drift, robust, and fast," 2015 IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA, 2015, pp. 2174-2181, doi: 10.1109/ICRA.2015.7139489.

주요 내용

1. 배경 및 동기

- LiDAR는 정확한 3D 거리 측정을 제공하지만, 스캔 속도가 느리고 고해상도 텍스처 정보를 얻기 어렵습니다.    
- 카메라는 고해상도 영상을 빠르게 제공하지만, 깊이 정보가 부족하고 조명 변화에 민감합니다.    
- 이 논문은 두 센서의 장점을 결합하여 상호 보완적인 시스템을 설계하고자 했습니다. 이는 "Visual LOAM"이라는 이름으로 불리며, 기존 LOAM(Lidar Odometry and Mapping)을 확장한 형태입니다.    

2. 방법론

논문은 두 가지 주요 모듈로 구성됩니다: **오도메트리(Odometry)**와 매핑(Mapping).

- Visual-LiDAR 오도메트리    
    - LiDAR 포인트 클라우드와 카메라 영상을 동시에 활용해 센서의 6-DoF(자유도 6) 움직임을 추정합니다.        
    - 카메라에서 추출한 특징점(feature points)을 LiDAR 데이터와 정합하여 정확도를 높입니다. 
    - 고속 모션 추정을 위해 두 센서 간의 시간 동기화와 캘리브레이션을 수행합니다.        
    - 이 모듈은 초당 10Hz 이상으로 동작하며, 실시간 처리가 가능합니다.
        
- 매핑    
    - LiDAR 데이터를 기반으로 3D 포인트 클라우드 맵을 생성하며, Visual 데이터로 텍스처와 세부 정보를 보강합니다.        
    - 오도메트리 결과를 활용해 누적 오차(drift)를 최소화합니다.        
    - 대규모 환경에서도 효율적으로 동작하도록 최적화되었습니다.        
- 기술적 특징    
    - 저드리프트: Visual 데이터가 LiDAR의 누적 오차를 보정하며, 특히 회전 및 직진 이동에서 안정성을 확보합니다.        
    - 견고함: 조명 변화나 동적 객체에도 강건하게 동작합니다.        
    - 속도: 실시간 처리를 위해 계산 효율성을 극대화했습니다.        

3. 알고리즘 구조

- 입력: LiDAR 포인트 클라우드와 카메라 영상.    
- 처리 단계:    
    1. 카메라에서 특징점 추출 (예: SIFT, ORB 등).        
    2. LiDAR 포인트 클라우드에서 에지(edge)와 평면(plane) 특징 추출.        
    3. 두 센서 데이터의 정합(matching) 및 모션 추정.        
    4. 맵 업데이트 및 최적화.        
- 출력: 로봇의 위치(오도메트리)와 3D 환경 맵.
    

4. 실험

- 환경: 실내 및 실외 환경에서 테스트 (예: 복도, 야외 도로).    
- 하드웨어: Velodyne LiDAR (HDL-32E)와 단안 카메라(monocular camera).    
- 결과:    
    - 순수 LiDAR 기반 LOAM 대비 드리프트 감소 (최대 50% 이상).        
    - 평균 위치 추정 오차: 약 1% 미만 (거리 대비).        
    - 처리 속도: 오도메트리 10Hz, 매핑 1Hz로 실시간 성능 확인.        
- 비교: 기존 Visual SLAM 및 LiDAR SLAM과 비교해 더 높은 정확도와 안정성을 보임.
    

기여도

1. 센서 융합: Visual과 LiDAR를 통합하여 단일 센서의 한계를 극복.    
2. 실시간 성능: 로봇 내비게이션에 바로 적용 가능한 속도와 효율성 제공.    
3. 확장성: LOAM 프레임워크를 기반으로 Visual 데이터를 추가해 범용성을 높임.    
4. 실용성: 자율 주행, 로봇 매핑 등 다양한 응용 분야에 기여.    


한계 및 논의

- 의존성: Visual 데이터의 품질(조명, 텍스처)에 따라 성능이 변동될 수 있음.    
- 복잡성: 두 센서의 캘리브레이션과 동기화가 필수적이며, 초기 설정이 까다로움.    
- 미래 과제: 동적 환경에서의 성능 개선 및 더 가벼운 연산 요구사항.
---

#### 16. 15 ICRA Initialization techniques for 3D SLAM  

L. Carlone, R. Tron, K. Daniilidis, and F. Dellaert, "Initialization techniques for 3D SLAM: A survey on rotation estimation and its use in pose graph optimization," in Proc. 2015 IEEE Int. Conf. Robot. Autom. (ICRA), Seattle, WA, USA, May 2015, pp. 4597-4604, doi: 10.1109/ICRA.2015.7139866.

---

1. 개요

- 목적: 이 논문은 3D SLAM(Simultaneous Localization and Mapping)에서 초기화 문제, 특히 회전 추정(rotation estimation)의 중요성을 조사하고, 이를 포즈 그래프 최적화(pose graph optimization)에 적용하는 방법을 다룹니다.
    
- 배경: SLAM은 로봇이 자신의 위치를 추정하고 동시에 환경 지도를 생성하는 기술로, 포즈 그래프 최적화는 이를 해결하는 핵심 비선형 문제입니다. 초기화가 부정확하면 최적화 과정에서 수렴 실패나 지역 최소(local minima)에 빠질 위험이 있습니다.
    
- 주요 주장: 회전 추정이 정확하면 비선형 문제를 선형 최소 제곱(linear least squares) 문제로 단순화할 수 있어, 계산 효율성과 견고성이 크게 향상됩니다.
    

---

2. 논문 구조

3. 서론 (Introduction): SLAM의 초기화 문제와 회전 추정의 필요성을 설명.
    
4. 포즈 그래프 최적화 개요 (Pose Graph Optimization): SLAM의 수학적 기반과 초기화의 역할.
    
5. 회전 추정 기술 (Rotation Estimation Techniques): 로봇 공학, 컴퓨터 비전, 제어 이론에서 사용되는 방법론 조사.
    
6. SLAM 초기화로의 적용 (Application to SLAM Initialization): 회전 추정 결과를 활용한 초기화 전략.
    
7. 실험 결과 (Experimental Results): 다양한 데이터셋에서의 성능 평가.
    
8. 결론 (Conclusion): 주요 발견과 향후 연구 방향.
    

---

3. 핵심 개념

	3.1 포즈 그래프 최적화

	- 포즈 그래프는 노드(로봇의 위치와 방향)와 엣지(노드 간 상대적 변환)로 구성된 그래프입니다.    
	- 목표는 모든 노드의 절대 포즈(위치와 회전)를 추정하여 측정값(엣지)과의 오차를 최소화하는 것입니다. 
	- 수학적 표현: $$\min_{x_1, ..., x_n} \sum_{(i,j) \in E} \| f(x_i, x_j) - z_{ij} \|^2$$    
	    여기서 $x_i$ 는 노드의 포즈(위치 $t_i$ , 회전 $R_i$ ), $z_{ij}$ 는 측정값, (f)는 포즈 변환 함수입니다.
    
    3.2 회전 추정의 중요성
	- 포즈는 위치(translation)와 회전(rotation)으로 구성되는데, 회전이 비선형성을 유발하는 주요 요인입니다.   
	- 회전이 알려져 있다면, 남은 위치 추정은 선형 문제가 되어 계산이 간단해집니다.    
	- 초기 회전 추정이 정확할수록 반복적 최적화(예: Gauss-Newton, Levenberg-Marquardt)의 수렴 속도와 성공률이 높아집니다.
    

---

4. 회전 추정 기술
   논문은 다양한 분야에서 사용되는 회전 추정 방법을 조사합니다:

	1. 컴퓨터 비전:
    
	    - PnP(Perspective-n-Point): 2D-3D 대응점을 이용해 카메라의 회전을 추정.       
	    - Essential Matrix: 스테레오 이미지 쌍에서 회전과 위치를 계산.        
	2. 로봇 공학:    
	    - IMU 통합: 자이로스코프 데이터를 활용한 회전 추정.        
	    - Odometry 기반: 휠 인코더나 비주얼 오도메트리로 초기 회전 계산.        
	3. 제어 이론:    
    - Wahba 문제: 벡터 관측쌍을 사용해 최적 회전을 구하는 문제(SVD 기반 해법 등).        

	- 논문은 이러한 방법들을 SLAM 초기화에 맞게 조정하고, 특히 Wahba 문제 기반 접근법이 계산 효율성과 정확도 측면에서 유리하다고 강조합니다.
    

---

5. SLAM 초기화 전략

	- 2단계 접근법:    
	    1. 회전 추정: 그래프의 모든 노드에 대해 초기 회전을 계산.        
	    2. 위치 추정: 회전을 고정하고 선형 최소 제곱으로 위치를 계산.        
	- 장점:    
	    - 비선형 최적화 전에 초기값을 제공하여 수렴 속도 개선.        
	    - 노이즈와 이상치(outlier)에 강인함.
        
	- 구현: 논문은 회전 추정을 위해 SVD(Singular Value Decomposition)를 활용하며, 이를 포즈 그래프에 적용.
    

---

6. 실험 결과

- 데이터셋: KITTI, TUM RGB-D 등 공개 데이터셋 사용.    
- 평가 기준:    
    - 초기화 후 최적화 수렴 시간.        
    - 최종 포즈 추정의 정확도(평균 제곱 오차, RMSE).        
- 결과:    
    - 제안된 초기화 방법은 기존 무작위 초기화(random initialization)나 단순 오도메트리 기반 방법보다 2-3배 빠른 수렴을 보임.        
    - 특히 큰 루프 폐쇄(loop closure)가 포함된 복잡한 환경에서 견고성 향상.        

---

7. 주요 발견 및 한계

- 발견:    
    - 회전 추정은 SLAM 초기화의 병목현상을 해결하는 핵심 열쇠.        
    - 선형화된 접근법으로 계산 비용을 줄이고, 대규모 문제에서도 적용 가능.        
- 한계:    
    - 이상치가 많은 경우 초기 회전 추정의 품질이 저하될 수 있음.        
    - 특정 센서(예: 저품질 IMU)에 의존할 경우 성능이 제한될 수 있음.
#### 17. 15 IROS NICP (dense normal)  

Serafin, J., & Grisetti, G. (2015). NICP: Dense normal based point cloud registration. In 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 742-749). IEEE. https://doi.org/10.1109/IROS.2015.7353455

1. 논문 개요

	배경
		점 구름 정합은 로봇 공학, 컴퓨터 비전, 3D 재구성 등에서 핵심 기술로, 두 개 이상의 점 구름 데이터를 정렬하여 일관된 좌표계로 변환하는 과정입니다. 기존의 Iterative Closest Point (ICP) 알고리즘은 점 대 점(Point-to-Point) 또는 점 대 평면(Point-to-Plane) 메트릭을 사용하여 정합을 수행했으나, 초기 정렬이 부정확하거나 노이즈가 많을 경우 성능이 저하되는 한계가 있었습니다. NICP는 이러한 문제를 해결하고자 **표면의 법선(normal)** 과 곡률(curvature) 정보를 활용하여 더 견고하고 정확한 정합을 목표로 합니다.

	목표
		- 실시간(online)으로 동작 가능한 점 구름 정합 알고리즘 개발    
		- 밀집된(dense) 데이터에서 법선 기반의 데이터 연관성(data association)을 개선    
		- 기존 ICP 대비 초기 정렬 의존도를 줄이고 계산 효율성을 유지
    
2. 방법론: NICP 알고리즘
	NICP는 표면의 기하학적 특성을 적극 활용하며, 다음과 같은 단계로 구성됩니다.
		2.1 데이터 연관성 (Data Association)
			- 기존 ICP와의 차이점: 전통적인 ICP는 가장 가까운 점 쌍을 기반으로 연관성을 찾지만, NICP는 법선 벡터와 곡률을 추가로 고려합니다. 이는 두 점 구름 간의 기하학적 일치성을 높여 잘못된 매칭을 줄입니다.    
			- 법선 계산: 각 점에 대해 국부적인 표면 법선을 추정합니다. 이는 보통 k-최근접 이웃(k-NN) 또는 주성분 분석(PCA)을 통해 수행됩니다.    
			- 곡률 고려: 법선 외에 곡률을 계산하여 평평한 영역과 곡면 영역을 구분합니다. 이는 데이터 연관성에서 가중치를 다르게 부여하는 데 사용됩니다. 
		2.2 오차 함수 정의
			NICP는 최소 제곱법(least-squares optimization)을 사용하여 두 점 구름 간의 변환(translation, rotation)을 추정합니다. 오차 함수는 다음과 같이 정의됩니다:
			- 점 대 평면 메트릭(Point-to-Plane Metric): 타겟 점 구름의 법선을 기준으로 소스 점의 거리를 최소화.    
			- 법선 일치성 가중치: 두 점의 법선 방향이 유사할수록 더 높은 가중치를 부여.    
			- 수식적으로는 다음과 같습니다 (논문에서 간략화된 형태로 제시):
	        $$E = \sum_{i} w_i \cdot || (p_i - T \cdot q_i) \cdot n_i ||^2$$
    여기서 $p_i$  는 타겟 점, $q_i$ 는 소스 점, $n_i$ 는 타겟 점의 법선, (T)는 변환 행렬, $w_i$ 는 가중치(법선 유사도 기반)입니다.    
		2.3 최적화
			- 비선형 최소 제곱 문제를 Levenberg-Marquardt 알고리즘으로 해결.
			- 초기 추정값이 없어도 수렴 가능성을 높이기 위해 다중 해상도(multi-resolution) 접근법을 선택적으로 사용.    
		2.4 실시간 처리
			- 계산 효율성을 위해 점 구름을 다운샘플링하거나, 가까운 점만을 대상으로 연관성을 계산하는 방식 적용.    

3. 실험 설정
	데이터셋

	- 공개 벤치마크: Stanford 3D Scanning Repository (예: Bunny 데이터셋)와 RGB-D SLAM 데이터셋을 사용.    
	- 실제 데이터: Kinect 센서를 통해 수집된 실내 환경 데이터.   
	비교 대상
	- 표준 ICP (Point-to-Point 및 Point-to-Plane)
    - Generalized ICP (GICP)
    - 3D-NDT (Normal Distributions Transform)
    
	평가 메트릭
	- 정확도: 정합 후 점 구름 간 평균 오차 (Root Mean Square Error, RMSE)
    - 수렴성: 초기 정렬이 틀어진 경우에도 성공적으로 정합되는 비율
    - 속도: 프레임당 처리 시간 (밀리초 단위)

4. 결과 분석
	4.1 성능
	- 정확도: NICP는 RMSE 기준으로 GICP와 비슷하거나 약간 우수한 결과를 보였으며, 표준 ICP 대비 월등히 개선됨.    
	- 수렴성: 초기 변환 오차가 큰 경우(예: 30도 이상 회전), NICP는 기존 ICP보다 20-30% 높은 성공률을 기록.    
	- 속도: 실시간 처리가 가능했으나 (약 50-100ms/프레임), GICP보다 약간 느린 것으로 나타남. 이는 법선 계산과 가중치 부여의 추가 연산 때문.    
	4.2 강점
	- 법선과 곡률을 활용한 데이터 연관성으로 노이즈와 이상치(outlier)에 강건함.
    - 초기 정렬이 부정확해도 수렴 가능성이 높음.
    - 밀집된 점 구름에서 특히 유리.
    4.3 한계
	- 법선 계산의 품질에 의존적: 노이즈가 심한 데이터에서는 성능 저하 가능.
    - 계산 복잡도가 기존 ICP보다 높아 저사양 하드웨어에서는 부담.
#### 18. 16 SSRR ICP-SLAM  

#### 19. 18 RSS SuMa (projective view rendering)  

#### 20. 18 ICRA IMLS-SLAM (feature selection + scan to map)  
#### 21. 18 ICRA Elastic LiDAR Fusion (map deformation)  
#### 22. 18 IROS LeGO-LOAM (LOAM + range image)  
#### 23. 18 IROS LIPS (plane)  
#### 24. 18 IROS Scan Context (robust pr)  
#### 25. 19 A-LOAM (code only)  
#### 26. 19 ICRA Lio-mapping (Lidar + IMU)  

#### 27. 19 IV Delio (rot/trans decoupled)  
#### 28. 19 IJRR SegMap (deep pr)  

#### 29. 19 IROS SuMa++ (semantic)  

#### 30 . 19 IROS Highway Laser-Inertial Odometry and Mapping (semantic + fusion)  

#### 31. 19 IROS Stereo Visual Inertial LiDAR fusion (짬뽕)

SuMa++와 PIN-SLAM은 모두 Xieyuanli Chen, Jens Behley, Cyrill Stachniss 등이 포함된 동일한 연구팀(주로 University of Bonn의 Photogrammetry and Robotics Lab)에서 개발된 LiDAR 기반 SLAM(Simultaneous Localization and Mapping) 기술입니다. 그러나 두 접근법은 목표, 방법론, 그리고 구현 방식에서 뚜렷한 차이점을 보입니다. 아래에서 주요 차이점을 상세히 설명하겠습니다.

---

1. 발표 시기와 연구 맥락

- SuMa++ (2019): 2019년 IROS에서 발표된 SuMa++는 기존 SuMa(Surfel-based Mapping)를 확장한 것으로, 의미론적 정보를 통합하여 LiDAR 기반 SLAM의 정확도와 효율성을 높이는 데 초점을 맞췄습니다. 당시 연구는 실시간 처리와 의미론적 이해를 강조하며 자율 주행 및 로봇 내비게이션에 적합한 솔루션을 목표로 했습니다.
    
- PIN-SLAM (2023): 2023년에 발표된 PIN-SLAM(Point-based Implicit Neural SLAM)은 이후 발전된 기술로, 신경망 기반 암묘적 표현(implicit representation)을 활용하여 더 컴팩트하고 유연한 지도 표현을 제공합니다. 이는 SuMa++ 이후 딥러닝과 신경 표현 기술의 발전을 반영한 결과물입니다.
    

---

2. 지도 표현 방식

- SuMa++:
    
    - 서펄(Surfel) 기반: SuMa++는 3D 점 구름을 작은 표면 요소(surfel)로 표현합니다. 각 서펄은 위치, 법선, 반지름, 그리고 의미론적 레이블(예: "도로", "차량")을 포함합니다.
        
    - 의미론적 통합: RangeNet++와 같은 신경망을 통해 점별 의미론적 레이블을 추출하고, 이를 서펄에 추가하여 지도에 환경의 의미를 반영합니다.
        
    - 특징: 기하학적 정보와 의미론적 정보를 결합한 명시적(explicit) 표현으로, 직관적이고 실시간 처리가 가능하지만 메모리 사용량이 상대적으로 큽니다.
        
- PIN-SLAM:
    
    - 암묘적 신경 표현(Implicit Neural Representation): PIN-SLAM은 점 기반의 암묘적 표현(Point-based Implicit Neural map, PIN)을 사용합니다. 이는 신경망(MLP: Multi-Layer Perceptron)을 통해 점 구름 데이터를 연속적인 함수로 모델링합니다.
        
    - 컴팩트성: 서펄처럼 개별 요소를 명시적으로 저장하지 않고, 신경망 파라미터로 환경을 압축적으로 표현하여 메모리 효율성이 높습니다.
        
    - 특징: 암묘적 표현은 더 유연하고 연속적인 지도 재구성을 가능하게 하며, 메시(mesh) 형태로 고품질 재구성이 가능합니다.
        

---

3. 의미론적 정보 활용

- SuMa++:
    
    - 의미론적 분할(semantic segmentation)을 통해 동적 객체(예: 차량, 보행자)를 필터링하고, 정적 환경에 대한 정합을 개선합니다.
        
    - 의미론적 레이블은 주로 데이터 연관성(data association)과 루프 클로저(loop closure)를 강화하는 데 사용됩니다.
        
- PIN-SLAM:
    
    - 의미론적 정보는 암묘적 표현에 통합되며, 환경의 기하학적 일관성과 함께 의미적 일관성을 유지하는 데 중점을 둡니다.
        
    - 동적 객체 제거보다는 전체 지도의 글로벌 일관성(global consistency)을 높이는 데 초점을 맞춥니다.
        

---

4. 루프 클로저와 최적화

- SuMa++:
    
    - 루프 클로저는 의미론적 일관성을 기반으로 탐지되며, projective ICP(Iterative Closest Point)를 활용해 스캔 정합을 개선합니다.
        
    - 최적화는 주로 로컬 수준에서 이루어지며, 실시간 성능을 우선시합니다.
        
- PIN-SLAM:
    
    - OverlapNet과 같은 이전 연구를 기반으로 루프 클로저를 탐지하며, 암묘적 표현을 통해 글로벌 최적화를 수행합니다.
        
    - 신경망 기반의 최적화로, 지도 전체의 일관성을 유지하며 더 정교한 후처리가 가능합니다.
        

---

5. 성능과 응용

- SuMa++:
    
    - 강점: 실시간 처리에 적합하며, 도시 환경이나 고속도로와 같은 대규모 환경에서 의미론적 지도를 생성하는 데 효과적입니다.
        
    - 한계: 서펄 기반 표현은 메모리 사용량이 크고, 동적 환경에서 완벽히 대응하기 어려울 수 있습니다.
        
    - 응용: 자율 주행 차량의 실시간 내비게이션, LiDAR 기반 로봇 매핑.
        
- PIN-SLAM:
    
    - 강점: 컴팩트한 지도 표현으로 메모리 효율성이 뛰어나며, 고품질 메시 재구성을 통해 시각화와 후처리에 유리합니다. 글로벌 일관성이 뛰어납니다.
        
    - 한계: 신경망 학습과 추론 과정이 필요해 계산 비용이 높을 수 있으며, 실시간성보다는 정밀도에 초점을 맞춥니다.
        
    - 응용: 장기적인 환경 매핑, 3D 재구성, 시뮬레이션.
        

---

6. 기술적 진화

- SuMa++는 전통적인 SLAM 기법(ICP, 서펄 표현)에 의미론적 딥러닝을 접목한 중간 단계의 연구로 볼 수 있습니다.
    
- PIN-SLAM은 SuMa++의 한계를 극복하고자 신경 표현(Neural Representation)과 같은 최신 딥러닝 기술을 도입하여, 더 현대적이고 미래 지향적인 접근법을 제시합니다.
    

---

요약 표

|특징|SuMa++ (2019)|PIN-SLAM (2023)|
|---|---|---|
|지도 표현|서펄 기반 (명시적)|암묘적 신경 표현 (PIN)|
|의미론적 활용|동적 객체 필터링, 정합 개선|글로벌 일관성 유지|
|루프 클로저|의미론적 ICP 기반 로컬 최적화|신경망 기반 글로벌 최적화|
|메모리 효율성|중간 (서펄 저장 필요)|높음 (압축된 신경 표현)|
|실시간성|강함|약함 (추론 비용 높음)|
|응용|자율 주행 실시간 매핑|장기 매핑, 고품질 3D 재구성|

---

결론

SuMa++와 PIN-SLAM은 동일한 연구팀의 연속적인 발전 과정을 보여줍니다. SuMa++는 실시간성과 의미론적 이해에 초점을 맞춘 반면, PIN-SLAM은 암묘적 표현을 통해 메모리 효율성과 글로벌 일관성을 극대화한 후속 연구입니다. 사용 목적에 따라 선택이 달라질 수 있는데, 실시간 내비게이션에는 SuMa++가, 정밀한 장기 매핑과 재구성에는 PIN-SLAM이 더 적합합니다.

추가로 특정 부분에 대해 더 알고 싶으시면 말씀해주세요!

https://jml-note.tistory.com/entry/Graph-SLAM-with-Example-Code

https://velog.io/@cjh1995-ros/SLAM-and-DL-Paper-Lists