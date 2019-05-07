# Use the docker container to run HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/


Build the HAROS docker image:
```
cd path-to-folder-that-contains-the-Dockerfile
sudo docker build --tag=haros .
```

Run the docker extractor script givig as argument the url of the GitHub repository that holds the source code:
```
sudo docker run -p 4000:4000 -ti haros:latest /haros_call.sh  *Repo_URL*
```

For example:
```
sudo docker run -p 4000:4000 -ti haros:latest /haros_call.sh https://github.com/ros/ros_tutorials/
```

Open on your browser the page: http://localhost:4000/
