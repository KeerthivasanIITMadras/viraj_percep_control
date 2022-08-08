# viraj_simulation_task
My gazebo world with viraj in it

![Screenshot from 2022-08-08 16-26-01](https://user-images.githubusercontent.com/94305617/183403191-eb2d7998-ff67-49b7-b33c-07e0de0b9ceb.png)

The detected potholes boundaries are mapped on RViz as pointclouds

The pixels generated from the contours are converted to ground frame distances wrt to the camera using inverse perspective mapping (this formula is used):

https://user-images.githubusercontent.com/94188928/183304762-854ea2c4-e679-457e-8b06-b730e416202c.png

Projecting pothole boundaries to rviz
![Screenshot from 2022-08-08 16-19-47](https://user-images.githubusercontent.com/94305617/183403297-652b2104-572a-4f29-9112-85c0d91d755c.png)
