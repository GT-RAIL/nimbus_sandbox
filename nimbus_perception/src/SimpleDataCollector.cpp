#include <nimbus_perception/SimpleDataCollector.h>

#include <iostream>
#include <fstream>

using namespace std;

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

SimpleDataCollector::SimpleDataCollector() : pnh("~")
{
  objects.objects.clear();

  currentObjectPublisher = pnh.advertise<sensor_msgs::PointCloud2>("object", 1);

  newDataSubscriber = n.subscribe<rail_manipulation_msgs::SegmentedObjectList>("rail_segmentation/segmented_objects", 1, &SimpleDataCollector::newDataCallback, this);
}

void SimpleDataCollector::newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData)
{
  boost::mutex::scoped_lock lock(objectMutex);

  if (newData->cleared)
    return;

  objects = *newData;
  index = 0;
  showObject(index);
}

void SimpleDataCollector::showObject(unsigned int index)
{
  //show object point cloud
  currentObjectPublisher.publish(objects.objects[index].point_cloud);

  //calculate minimum area bounding box
  //convert point cloud to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(objects.objects[index].point_cloud, converter);
  pcl::fromPCLPointCloud2(converter, *objectCloud);

  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  centroid[0] = objects.objects[index].centroid.x;
  centroid[1] = objects.objects[index].centroid.y;
  centroid[2] = objects.objects[index].centroid.z;
  pcl::computeCovarianceMatrixNormalized(*objectCloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  //move the points to that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block(0, 0, 3, 3) = eig_dx.transpose();
  p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
  pcl::PointCloud<pcl::PointXYZRGB> c_points;
  pcl::transformPointCloud(*objectCloud, c_points, p2w);

  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(c_points, min_pt, max_pt);

  //sort from least to greatest
  // vector<float> dims;
  dims.push_back(fabs(max_pt.x - min_pt.x));
  dims.push_back(fabs(max_pt.y - min_pt.y));
  dims.push_back(fabs(max_pt.z - min_pt.z));
  sort(dims.begin(), dims.end());

  Eigen::Vector3f rgb;
  rgb[0] = objects.objects[index].marker.color.r;
  rgb[1] = objects.objects[index].marker.color.g;
  rgb[2] = objects.objects[index].marker.color.b;
  // Eigen::Vector3f lab = RGB2Lab(rgb);
  lab = RGB2Lab(rgb);

  //display object info
  ROS_INFO("--------------------------------------------");
  ROS_INFO("Showing data for object %d", index);
  ROS_INFO("RGB: %f, %f, %f", objects.objects[index].marker.color.r, objects.objects[index].marker.color.g, objects.objects[index].marker.color.b);
  ROS_INFO("LAB: %f, %f, %f", lab[0], lab[1], lab[2]);
  ROS_INFO("Axis-aligned bounding box: %f, %f, %f", objects.objects[index].width, objects.objects[index].height, objects.objects[index].depth);
  ROS_INFO("Min area bounding box: %f, %f, %f", dims[0], dims[1], dims[2]);
  ROS_INFO("Center: %f, %f, %f", objects.objects[index].center.x, objects.objects[index].center.y, objects.objects[index].center.z);
  ROS_INFO("Size heuristic: %f", sqrt(pow(objects.objects[index].width,2) + pow(objects.objects[index].height,2) + pow(objects.objects[index].depth,2)));
  ROS_INFO("YAML entry: ");
  cout << "- l: " << lab[0] << endl;
  cout << "  a: " << lab[1] << endl;
  cout << "  b: " << lab[2] << endl;
  cout << "  x: " << dims[0] << endl;
  cout << "  y: " << dims[1] << endl;
  cout << "  z: " << dims[2] << endl;
  cout << "  label: " << endl;
}

void SimpleDataCollector::loop()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("      Reading from Keyboard      ");
  puts("---------------------------------");
  puts(" Press Q/W to cycle back/forward ");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

    //Display help message
    if (c == KEYCODE_Q)
    {
      boost::mutex::scoped_lock lock(objectMutex);
      if (index != 0)
      {
        index --;
        showObject(index);
      }

    }
    else if (c == KEYCODE_W)
    {
      boost::mutex::scoped_lock lock(objectMutex);
      if (!objects.objects.empty() && index < objects.objects.size() - 1)
      {
        index ++;
        showObject(index);
      }
    }
    // Save captured values
    //else if (c == KEYCODE_ENTER)
    else
    {
      string label;

      cout << "Please enter label: " <<endl;
      cin >> label;

      // write into csv
      ofstream file;
      file.open("dataset.csv", ofstream::out | ofstream::app);
      file << lab[0] << "," << lab[1] << "," << lab[2] << "," << dims[0] << "," << dims[1] << "," << dims[2] << "," << label << "\n";
      file.close();

      cout << label << " Saved!" << endl;
      cout << "Press Q or W to continue navigating." << endl;
    }
  }
}

void shutdown(int sig)
{
  // shut everything down
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB)
{
  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double R, G, B, X, Y, Z;

  R = colorRGB[0];
  G = colorRGB[1];
  B = colorRGB[2];

  // linearize sRGB values
  if (R > 0.04045)
    R = pow ( (R + 0.055) / 1.055, 2.4);
  else
    R = R / 12.92;

  if (G > 0.04045)
    G = pow ( (G + 0.055) / 1.055, 2.4);
  else
    G = G / 12.92;

  if (B > 0.04045)
    B = pow ( (B + 0.055) / 1.055, 2.4);
  else
    B = B / 12.92;

  // postponed:
  //    R *= 100.0;
  //    G *= 100.0;
  //    B *= 100.0;

  // linear sRGB -> CIEXYZ
  X = R * 0.4124 + G * 0.3576 + B * 0.1805;
  Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
  Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

  // *= 100.0 including:
  X /= 0.95047;  //95.047;
  //    Y /= 1;//100.000;
  Z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (X > 0.008856)
    X = pow (X, 1.0 / 3.0);
  else
    X = 7.787 * X + 16.0 / 116.0;

  if (Y > 0.008856)
    Y = pow (Y, 1.0 / 3.0);
  else
    Y = 7.787 * Y + 16.0 / 116.0;

  if (Z > 0.008856)
    Z = pow (Z, 1.0 / 3.0);
  else
    Z = 7.787 * Z + 16.0 / 116.0;

  Eigen::Vector3f colorLab;
  colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
  colorLab[1] = static_cast<float> (500.0 * (X - Y));
  colorLab[2] = static_cast<float> (200.0 * (Y - Z));

  return colorLab;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "simple_data_collector");

  // initialize the keyboard controller
  SimpleDataCollector sdc;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread myThread(boost::bind(&SimpleDataCollector::loop, &sdc));
  ros::spin();

  // wait for everything to end
  myThread.interrupt();
  myThread.join();
}