#include <array>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{

struct Options
{
  std::string base;
  std::string increment;
  std::string output;
  std::string transform;
  float leaf{0.1F};
};

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string key{argv[index]};
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + key);
    }
    const std::string value{argv[++index]};
    if (key == "--base") {
      options.base = value;
    } else if (key == "--increment") {
      options.increment = value;
    } else if (key == "--output") {
      options.output = value;
    } else if (key == "--transform") {
      options.transform = value;
    } else if (key == "--leaf") {
      options.leaf = std::stof(value);
    } else {
      throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (options.base.empty() || options.increment.empty() || options.output.empty() ||
    options.transform.empty())
  {
    throw std::invalid_argument(
            "--base, --increment, --output and --transform are required");
  }
  if (!(options.leaf > 0.0F)) {
    throw std::invalid_argument("--leaf must be positive");
  }
  return options;
}

Eigen::Matrix4f parse_transform(const std::string & csv)
{
  std::array<float, 16> values{};
  std::stringstream stream{csv};
  std::string item;
  std::size_t index{0};
  while (std::getline(stream, item, ',')) {
    if (index >= values.size()) {
      throw std::invalid_argument("transform must contain exactly 16 values");
    }
    values[index++] = std::stof(item);
  }
  if (index != values.size()) {
    throw std::invalid_argument("transform must contain exactly 16 values");
  }
  Eigen::Matrix4f matrix;
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
        values[row * 4 + column];
    }
  }
  return matrix;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options{parse_options(argc, argv)};
    auto base = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto increment = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(options.base, *base) < 0) {
      throw std::runtime_error("failed to load base PCD: " + options.base);
    }
    if (pcl::io::loadPCDFile(options.increment, *increment) < 0) {
      throw std::runtime_error("failed to load increment PCD: " + options.increment);
    }

    auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*increment, *aligned, parse_transform(options.transform));
    *base += *aligned;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(options.leaf, options.leaf, options.leaf);
    voxel.setInputCloud(base);
    pcl::PointCloud<pcl::PointXYZ> merged;
    voxel.filter(merged);
    if (pcl::io::savePCDFileBinaryCompressed(options.output, merged) < 0) {
      throw std::runtime_error("failed to save merged PCD: " + options.output);
    }
    std::cout << "merged " << base->size() << " input points into "
              << merged.size() << " voxels\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "merge_pcd_maps: " << error.what() << '\n';
    return 2;
  }
}
