# #!/bin/bash
# git clone -b marco-sg-control git@github.com:MerlinLaboratory/abb_librws_2.0.git
# git clone -b cari git@github.com:MerlinLaboratory/abb_omnicore_ros2.git
#docker buildx build --no-cache --platform linux/amd64,linux/arm64 --network=host --ssh default -t smentasti/drims2:2025 --push . 
docker build  --network=host --ssh default -t gabrinovas/drims2:2025 --push . 

