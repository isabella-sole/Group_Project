**Obstacle detection and tracking in maritime environments**

*For further information about the project read the "gp_report.pdf" document.*

*1. Motivations and general objectives:*

Environment perception plays a vital role in autonomous systems. In this project, scene
understanding in maritime environments is tackled using an obstacle detection and tracking
procedure based on a video camera and a LIDAR (range sensor).
The current state of the project includes a video analysis prototype able to detect vessels with a
state-of-the-art object detection CNN (Convolutional neural network) YOLO that detect obstacles in
the image plane. We also implemented an efficient object tracking procedure to track objects across
image frames. Concerning the LIDAR, point clouds are clustered and tracked in 3D, with a Kalman
filter estimating the object’s size and position.
The general objective is to reinforce the link between the vision and LIDAR processing pipelines.
For example, the point cloud of an obstacle boat acquired by the LIDAR may vary quite a lot due
due to the sea motion and the tilt angles of the LIDAR sensor. The bounding box of the same boat,
as estimated by the CNN, projected in the point cloud might allow to recover a more precise size of
the boat. The vision clues could also be exploited to perform better data association in case of
multiple obstacles.

*2.Proposed work plan:*

•Enhance the 3D points clustering algorithm with information coming from the camera (e.g.,
the bounding box in the image plane generates a cone in the 3D cloud, which is used for
segmentation and clustering)

•Enhance the 3D points clustering algorithm fusing multiple LIDAR acquisitions

•Expand the software to multiple obstacles, managing data association (again visual
information can be used to guide data association)

•Increase the size of annotated data

*3. Results:*

•Ground truth comparison with manually annotated data, assessing:

◦ The advantages of the proposed improvements of the clustering algorithm

◦ The stability of the obstacle’s bounding box size, heading and velocity estimation


**To run the project**

In order to run the project dowload the folder, compile it inside its workspace with the command:

$ catkinmake

In a terminal run the ROSMASTER:

$ roscore

and in a second terminal launch the code once inside the ws:

$ roslaunch tracking_lidar trackdim.launch


**Authors**

Isabella-Sole Bisio: *bisioisabellasole@gmail.com*

Serena Roncagliolo: *serena.roncagliolo@gmail.com*

