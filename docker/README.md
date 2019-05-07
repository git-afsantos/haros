# Use the docker container to run HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/


Build the HAROS docker image:
```
cd path-to-folder-that-contains-the-Dockerfile
[sudo] docker build --tag=haros .
```


Create a docker container, copy the `index.yaml` file in its filesystem and commit the changes as a new image

```
[sudo] docker create -ti --name *Container name* haros
[sudo] docker cp index.yaml *Container name*:/root/.haros/
[sudo] docker commit *Container name* *Image name*
```


Run the newly created image wiht the docker extractor script givig as argument the url of the GitHub repository that holds the source code:
```
[sudo] docker run -p 4000:4000 -ti *Image name* /haros_call.sh  *Repo_URL*
```

For example:
```
[sudo] docker create -ti --name haros_container haros
[sudo] docker cp index.yaml haros_container:/root/.haros/
[sudo] docker commit haros_container haros-roscpp-tutorials
[sudo] docker run -p 4000:4000 -ti haros-roscpp-tutorials:latest /haros_call.sh https://github.com/ros/ros_tutorials/
```

Open on your browser the page: http://localhost:4000/
