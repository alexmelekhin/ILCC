#!/bin/bash

docker exec --user docker_ilcc -it ilcc \
    /bin/bash -c "cd /home/docker_ilcc; echo ilcc container; echo ; /bin/bash"
