xtion_grabber - ROS nodelet for xtion kernel driver
===================================================

This package contains a nodelet for use with the [xtion][] kernel driver. It
configures the driver according to parameters and publishes color and depth
images in the standard `sensor_msgs::Image` format.

It also advertises a `sensor_msgs::PointCloud2` topic with RGB-D point clouds
generated from the depth and color images.

License
-------

`xtion_grabber` is currently licensed under GPLv2. If this restricts your usage,
please let me know and we might be able to figure something out.

[xtion]: https://github.com/xqms/xtion
