I was able to complete the "Demo unsupervised image segmentation" (see vklemm_hw5_demo.png).

But next, I was not able to run the container: I get the same error as several people (as specified on slack) with the following error message:

My command:
docker -H dux.local run -it --memory="800m" --memory-swap="1.8g" --net host --privileged -v /data:/data --name kmeans duckietown/devel-kmeans-unsup /bin/bash

My error:
standard_init_linux.go:190: exec user process caused "exec format error"
