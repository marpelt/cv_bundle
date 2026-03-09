# computer-vision-bundle-robocup

A modular computer vision bundle designed for RoboCup. The package is fully topic-based and integrates seamlessly into existing ROS workflows.

Make sure all required dependencies are installed in your ROS environment. Before running the nodes, ensure that all scripts inside the packages are executable.

To explore and understand available parameters, take a look at:
```launch/test.launch```

The only external requirement is a tool to visualize image streams, such as:

- rqt_image_view
- RViz
- Any compatible image topic viewer


```bash
git clone --recursive git@github.com:marpelt/cv_bundle.git
```
