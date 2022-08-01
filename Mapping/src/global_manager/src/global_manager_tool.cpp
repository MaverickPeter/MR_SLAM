#include <global_manager/global_manager.h>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

using namespace global_manager;

int main(int argc, char **argv)
{
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  std::string output_name = "output.pcd";

  if (pcd_file_indices.size() < 2) {
    pcl::console::print_error("Need at least 2 input files!\n");
    return -1;
  }

  const MapMergingParams params = MapMergingParams::fromCommandLine(argc, argv);
  std::cout << "params: " << std::endl << params << std::endl;

  // load input pointclouds
  std::vector<PointCloudConstPtr> clouds;
  for (int idx : pcd_file_indices) {
    PointCloudPtr cloud(new PointCloud);
    auto file_name = argv[idx];
    if (pcl::io::loadPCDFile<PointT>(file_name, *cloud) < 0) {
      pcl::console::print_error("Error loading pointcloud file %s. Aborting.\n",
                                file_name);
      return -1;
    }
    clouds.push_back(cloud);
  }

  pcl::console::print_highlight("Estimating transforms.\n");

  std::vector<Eigen::Matrix4f> transforms =
      estimateMapsTransforms(clouds, params);

  pcl::console::print_highlight("Estimated transforms:\n");

  for (const auto &transform : transforms) {
    std::cout << transform << std::endl;
  }

  pcl::console::print_highlight("Compositing clouds and writing to "
                                "output.pcd\n");

  PointCloudPtr result =
      composeMaps(clouds, transforms, params.output_resolution);

  pcl::io::savePCDFileBinary(output_name, *result);

  return 0;
}
