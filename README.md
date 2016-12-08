# Drone-Indoor-Navigation

This work presents an Unmanned Aerial Vehicle (UAV), based on the AR. Drone platform, which can perform an autonomous navigation in indoor (e.g. corridor, hallway) and industrial environments (e.g. production line). It also has the ability to avoid pedestrians while they are working or walking in the vicinity of the robot. The only sensor in our system is the front camera. For the navigation part our system rely on the vanishing point algorithm, the Hough transform for the wall detection and avoidance, and the HOG descriptors for pedestrian detection using SVM classifier. Our experiments show that our vision navigation procedures are reliable and enable the aerial vehicle to y without humans intervention and coordinate together in the same workspace. We are able to detect human motion with high confidence of 85% in a corridor and to confirm our algorithm in 80% successful fight experiments.

Link to the paper:
https://www.researchgate.net/publication/289834483_Safety_challenges_in_using_ARDrone_to_collaborate_with_humans_in_indoor_environments
