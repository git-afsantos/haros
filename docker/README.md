# Use the docker container to run HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image:
```
cd path-to-folder-that-contains-the-Dockerfile
[sudo] docker build --tag=haros .
```

Run the image with the docker extractor script givig as argument the url of the GitHub repository that holds the source code and mounting the `index.yaml` file to its filesystem:

Either
```
[sudo] docker run -it --mount type=bind,source=*Path to index.yaml*,target=/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh *Repo URL*
```

or

```
[sudo] docker run -it -v *Path to index.yaml*:/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh *Repo URL*
```

can be used.

### Examples
```
[sudo] docker run -it --mount type=bind,source="$(pwd)"/index.yaml,target=/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh https://github.com/ros/ros_tutorials/
```

or

```
[sudo] docker run -it -v "$(pwd)"/index.yaml:/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh https://github.com/ros/ros_tutorials/
```
Open on your browser the page: http://localhost:4000/
