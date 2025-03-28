
|     방식      |  연도  |  차원   | Map 저장 방식   | 기타  |
| :---------: | :--: | :---: | ----------- | --- |
|  FastSLAM   |      |  2D   |             |     |
|    LOAM     | 2014 |  3D   | Point Cloud |     |
|    iSAM     | 2008 |       |             |     |
|   LIO SAM   | 2020 | 2D+3D |             |     |
| ScanContext | 2021 |  2D   | Voxel+SCD   |     |
|  PIN SLAM   | 2024 | 2D+3D | Voxel+SDF   |     |
|             |      |       |             |     |


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