

#  Fast Lio 

2020년 Fast-LIO 논문이 발표된 뒤, 
2022년 Fast-LIO2가 발표되었고, 
이후 2024년 Fast-LIVO 가 발표되었다. 

24년 내용에선 Solid State LiDAR와 Vision 을 결합하여 QRB5165급 하드웨어에서 3차원 SLAM 및 환경 재현을 해낸 것이 특기할만한 내용이다. 

SLAM의 기본은 LiDAR SLAM의 기초에 기반하고 있는데, iKF 필터를 이용하여 한다는 부분이다. 

iKF 에서 중요한 부분은 iteration의 반복 조건을 어떻게 설정할 것이냐인데, Fast-LIO 1차 논문을 보면 그 해답이 나와있다. 

![[Pasted image 20250401222245.png]]

![[Pasted image 20250401204825.png]]


가만. 리그룹으로 올린 이유가 dt를 극도로 작게 가져가려고 이 짓거리 한건가...? 
.
어쨌거나. Fast LIO 1에서는 State Estimaition만 참고하면 될 것 같다. 

칼만 gain 구하는 계산식의 속도를 비약적으로 끌어올렸는데, Feature갯수가 늘어도 
계산시간의 증가가 낮다. 
![[Pasted image 20250401211730.png]]
10Hz로 LiDAR를 돌렸을때 다른 경쟁 알고리즘과 대비하여 그렇게 드라마틱하게 빠르진 않는데. 왜일까. 
![[Pasted image 20250401211839.png]]

어쨌건, 첫번째 논문에선 낮은 시스템 사양에도 불구하고 잘 돌아간다 정도의 임팩트만 주고 있다. 
![[Pasted image 20250401211928.png]]


# Fast LIO2

기존 논문에선 연산능력은 빨라졌지만 LOAM같은 알고리즘 대비하여 더 그만큼 빠르다는 이점을 주진 못했다. 이걸 어떻게 극복할까 고민했을 것이다. 


![[Pasted image 20250401222205.png]]