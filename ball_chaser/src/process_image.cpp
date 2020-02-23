#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

enum class Direction
{
  LEFT,
  FRONT,
  RIGHT
};

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget drive_to_service;
  drive_to_service.request.linear_x = lin_x;
  drive_to_service.request.angular_z = ang_z;

  if (!client.call(drive_to_service))
  {
    ROS_ERROR("Failed to call drive to service");
  }
}

bool locate_ball(const sensor_msgs::Image& img, std::size_t* pixel_position)
{
  int white_pixel = 255;

  // Loop through each pixel to find the first white one
  for (std::size_t i = 0; i < img.height * img.step; i = i + 3)
  {
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel &&
        img.data[i + 2] == white_pixel)
    {
      *pixel_position = i;
      return true;
    }
  }

  return false;
}

Direction estimate_direction(const sensor_msgs::Image& img,
                             std::size_t pixel_byte_position)
{
  // Calculate pixel position in the image
  std::size_t pixel_position = pixel_byte_position / (img.step / img.width);
  // Calculate pixel x-position in the image
  std::size_t x_pos_on_screen = pixel_position % img.width;

  if (x_pos_on_screen <= double(img.width) / 3.)
  {
    return Direction::LEFT;
  }
  if (x_pos_on_screen > double(img.width) / 3. &&
      x_pos_on_screen <= (2. / 3.) * double(img.width))
  {
    return Direction::FRONT;
  }
  if (x_pos_on_screen > (2. / 3.) * double(img.width))
  {
    return Direction::RIGHT;
  }
}

void drive_to(Direction direction)
{
  switch (direction)
  {
    case Direction::LEFT:
    {
      drive_robot(0.5, 0.5);
      break;
    }
    case Direction::FRONT:
    {
      drive_robot(0.5, 0);
      break;
    }
    case Direction::RIGHT:
    {
      drive_robot(0.5, -0.5);
      break;
    }
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  std::size_t pixel_byte_position = 0;
  if (!locate_ball(img, &pixel_byte_position))
  {
    ROS_INFO("404: Ball not found.");
    drive_robot(0, 0);
    return;
  }

  drive_to(estimate_direction(img, pixel_byte_position));
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  ROS_INFO("Processing image");
  // Handle ROS communication events
  ros::spin();

  return 0;
}