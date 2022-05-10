# pointcloud를 이용한 Lidar clsuter tracking 방법

# 본 코드는 다음의 순서로 개발되었음
# 본 방법이 정답은 아니므로, 참고만하고 더 좋은 방법에 대해 연구해보기 바람.
 
1. ROI 설정 하여 포인트 영역제한
2. Voxelization을 통해 voxel화 및 point 개수 줄이기
3. Point Clustering 진행
   1. 본 코드에서는 Euclid 방식을 사용하여 Clustering을 진행
   2. DBSCAN 등 다양한 방법으로 자신의 task에 맞는 방법을 사용하면 됨
4. Cluster를 통해 추출된 Cluster에서 배경 분리
   1. 이점은 자신만의 방법 사용
   2. ex) point가 매우 많이 뭉쳐 있는 경우 배경으로 생각하고 제거
   3. 여기서는 min max를 통해 이점을 어느정도 해결
5. 이전 프레임에 존재하는 클러스터와 현재 프레임에 존재하는 클러스터가 같은 클러스터인지 비교
   1. 이전 프레임 A cluster가 현재 프레임에서 A cluster라고 판단된 클러스터와 같은 클러스터인가?
   2. 이걸 하는 이유는 칼만필터를 적용하기 위한 작업
   3. 칼만필터가 이전과 현재를 토대로 보정하는 방법이므로...
6. 칼만필터로 클러스터 중앙 위치값 보정
7. 이 과정을 계속 반복하여 계속 Tracking 진행
8. 현재 코드는 좋은 코드는 아니므로, Refactoring해서 더 좋은 코드를 설계하기 바람

도움될만한 사이트
https://pcl.gitbook.io/tutorial/part-0/part00-chapter01
https://pointclouds.org/

