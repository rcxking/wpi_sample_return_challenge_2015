# this message provides the information to take the pixel location of an object
# in an image and to convert that to a line in 3d space
# WARNING: make sure to use rectified images
std_msgs/Header header # use header from image
float64[2] point #xy pixel location of observation
float64[12] P # intrinsic for rectified image, from camera_info
