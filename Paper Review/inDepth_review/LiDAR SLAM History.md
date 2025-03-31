
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

3월 목표 1: lidar slam history 정리 (각 5줄 이내)  
- 87 PAMI Least-squares fitting of two 3D point sets  
- 92 PAMI ICP  
- 97 AR Globally consistent range scan alignment  
- 03 IROS NDT registration  
- 06 IJRR Square Root SAM  
- 07 JFR 3D NDT registration  
- 08 TRO iSAM  
- 09 ICRA Olson (correlative scan matching)  
- 09 ICRA FPFH registration  
- 09 RSS GICP  
- 10 ITSM A Tutorial on Graph-Based SLAM  
- 11 IV Velodyne SLAM  
- 12 TRO Zebedee  
- 12 RAM PCL tutorial  
- 13 ICRA DVO (lidar 는 아니지만)  
- 13 AR Libpointmatcher  
- 14 RSS LOAM (feature matching + scan to map)  
- 15 ICRA VLOAM (visual + LOAM)  
- 15 ICRA Initialization techniques for 3D SLAM  
- 15 RAM Registration tutorial (Registration with the Point cloud library)  
- 15 IROS NICP (dense normal)  
- 16 SSRR ICP-SLAM  
- 16 book World modeling  
- 18 RSS SuMa (projective view rendering)  
- 18 ICRA IMLS-SLAM (feature selection + scan to map)  
- 18 ICRA Elastic LiDAR Fusion (map deformation)  
- 18 IROS LeGO-LOAM (LOAM + range image)  
- 18 IROS LIPS (plane)  
- 18 IROS Scan Context (robust pr)  
- 19 A-LOAM (code only)  
- 19 ICRA Lio-mapping (Lidar + IMU)  
- 19 IV Delio (rot/trans decoupled)  
- 19 IJRR SegMap (deep pr)  
- 19 IROS SuMa++ (semantic)  
- 19 IROS Highway Laser-Inertial Odometry and Mapping (semantic + fusion)  
- 19 IROS Stereo Visual Inertial LiDAR fusion (짬뽕)
https://jml-note.tistory.com/entry/Graph-SLAM-with-Example-Code

https://velog.io/@cjh1995-ros/SLAM-and-DL-Paper-Lists