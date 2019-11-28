FROM ros3
RUN apt-get install -y tmux
RUN apt-get install -y libsm6 libxrender1 libfontconfig1
RUN pip3 install tqdm opencv-python onnxruntime
RUN pip3 install matplotlib

RUN pip3 install rospkg

COPY . /code
WORKDIR /code

# CMD ["python3", "predict_server.py"]
CMD ["/bin/bash", "-c", "source /ros_catkin_ws/devel/setup.bash && python3 ros_node.py"]