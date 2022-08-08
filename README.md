# viraj_simulation_task
My gazebo world with viraj in it

![Screenshot from 2022-08-08 16-26-01](https://user-images.githubusercontent.com/94305617/183403191-eb2d7998-ff67-49b7-b33c-07e0de0b9ceb.png)
The right camera shows image like, which is executed with stereo camera plugin:

![Screenshot from 2022-08-08 16-36-53](https://user-images.githubusercontent.com/94305617/183404836-13a86d43-71d5-4551-9685-88734e779fa2.png)


The detected potholes boundaries are mapped on RViz as pointclouds

The pixels generated from the contours are converted to ground frame distances wrt to the camera using inverse perspective mapping (this formula is used):

![Screenshot from 2022-08-08 16-34-09](https://user-images.githubusercontent.com/94305617/183404275-5cccdc05-60c6-4b90-8699-c894bcb472f1.png)

Projecting pothole boundaries to rviz
![Screenshot from 2022-08-08 16-19-47](https://user-images.githubusercontent.com/94305617/183403297-652b2104-572a-4f29-9112-85c0d91d755c.png)
