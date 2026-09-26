#include "pointmatcher/PointMatcher.h"
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using PM = PointMatcher<float>;
using DP = PM::DataPoints;
using Matrix = PM::TransformationParameters;

// libpointmatcher 1.4.4 cannot load binary PCD; our exporter writes float32 PCD.
static DP load_binary_pcd(const std::string &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot open PCD: " + path);
  std::vector<std::string> fields, types;
  std::vector<int> sizes, counts;
  size_t points = 0;
  std::string line;
  bool binary = false;
  while (std::getline(stream, line)) {
    std::istringstream row(line);
    std::string key;
    row >> key;
    if (key == "FIELDS") { std::string value; while (row >> value) fields.push_back(value); }
    if (key == "TYPE") { std::string value; while (row >> value) types.push_back(value); }
    if (key == "SIZE") { int value; while (row >> value) sizes.push_back(value); }
    if (key == "COUNT") { int value; while (row >> value) counts.push_back(value); }
    if (key == "POINTS") row >> points;
    if (key == "DATA") { std::string value; row >> value; binary = value == "binary"; break; }
  }
  if (!binary || !points || fields.size() != sizes.size() || fields.size() != types.size())
    throw std::runtime_error("Expected a nonempty binary PCD with FIELDS, SIZE, TYPE, POINTS");
  if (counts.empty()) counts.assign(fields.size(), 1);
  if (counts.size() != fields.size()) throw std::runtime_error("Invalid PCD COUNT");
  std::vector<size_t> offsets(fields.size());
  size_t stride = 0;
  int x = -1, y = -1, z = -1, intensity = -1;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (sizes[i] != 4 || types[i] != "F" || counts[i] != 1)
      throw std::runtime_error("Only float32 scalar PCD fields are supported");
    offsets[i] = stride;
    stride += 4;
    if (fields[i] == "x") x = i;
    if (fields[i] == "y") y = i;
    if (fields[i] == "z") z = i;
    if (fields[i] == "intensity") intensity = i;
  }
  if (x < 0 || y < 0 || z < 0) throw std::runtime_error("PCD is missing XYZ");
  std::vector<char> bytes(points * stride);
  stream.read(bytes.data(), bytes.size());
  if (static_cast<size_t>(stream.gcount()) != bytes.size())
    throw std::runtime_error("Truncated PCD payload");
  DP::Labels labels;
  for (const char *name : {"x", "y", "z", "pad"}) labels.push_back(DP::Label(name, 1));
  PM::Matrix feature(4, points);
  PM::Matrix intensities(1, points);
  for (size_t i = 0; i < points; ++i) {
    auto value = [&](int field) { float number; std::memcpy(&number, bytes.data() + i * stride + offsets[field], 4); return number; };
    feature(0, i) = value(x); feature(1, i) = value(y); feature(2, i) = value(z);
    feature(3, i) = 1.f;
    if (intensity >= 0) intensities(0, i) = value(intensity);
  }
  DP cloud(feature, labels);
  if (intensity >= 0) cloud.addDescriptor("intensity", intensities);
  return cloud;
}

int main(int argc, char **argv) {
  if (argc != 12 && argc != 13) {
    std::cerr << "Usage: icp_localize MAP.pcd SCAN.pcd CONFIG.yaml x y z qx qy qz qw OUT.json [ALIGNED_PREFIX]\n";
    return 2;
  }
  try {
    const DP reference = load_binary_pcd(argv[1]);
    DP reading = load_binary_pcd(argv[2]);
    if (reference.getEuclideanDim() != 3 || reading.getEuclideanDim() != 3)
      throw std::runtime_error("Map and scan must be 3D PCD clouds");
    const int source_count = reading.features.cols();
    const int map_count = reference.features.cols();
    if (source_count < 10 || map_count < 10)
      throw std::runtime_error("Map or scan is empty");

    // LeGO featureAssociation.cpp: raw RoboSense (x,y,z) -> camera (y,z,x).
    const auto raw = reading.features.topRows(3).eval();
    reading.features.row(0) = raw.row(1);
    reading.features.row(1) = raw.row(2);
    reading.features.row(2) = raw.row(0);

    Matrix initial = Matrix::Identity(4, 4);
    initial.block<3, 1>(0, 3) = Eigen::Vector3f(std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6]));
    Eigen::Quaternionf q0(std::stof(argv[10]), std::stof(argv[7]), std::stof(argv[8]), std::stof(argv[9]));
    if (std::abs(q0.norm() - 1.f) > 0.05f) throw std::runtime_error("Initial quaternion is not unit length");
    q0.normalize();
    initial.block<3, 3>(0, 0) = q0.toRotationMatrix();

    PM::ICP icp;
    std::ifstream config(argv[3]);
    if (!config) throw std::runtime_error("Cannot open ICP YAML");
    icp.loadFromYaml(config);
    if (argc == 13) {
      DP before = reading;
      icp.transformations.apply(before, initial);
      before.save(std::string(argv[12]) + "_before.pcd");
    }
    const auto start = std::chrono::steady_clock::now();
    const Matrix result = icp(reading, reference, initial);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (!result.allFinite()) throw std::runtime_error("ICP returned a nonfinite transform");
    Eigen::Quaternionf q(result.block<3, 3>(0, 0));
    q.normalize();
    if (argc == 13) {
      DP aligned = reading;
      icp.transformations.apply(aligned, result);
      aligned.save(std::string(argv[12]) + "_after.pcd");
    }
    int iterations = -1;
    for (const auto &checker : icp.transformationCheckers) {
      if (checker->className == "CounterTransformationChecker") {
        const auto vars = checker->getConditionVariables();
        if (vars.size()) iterations = static_cast<int>(std::round(vars[0]));
      }
    }
    const double weighted_ratio = icp.errorMinimizer->getWeightedPointUsedRatio();
    std::ofstream out(argv[11]);
    if (!out) throw std::runtime_error("Cannot open output JSON");
    out << std::setprecision(9) << "{\n"
        << "  \"x\": " << result(0, 3) << ", \"y\": " << result(1, 3) << ", \"z\": " << result(2, 3) << ",\n"
        << "  \"qx\": " << q.x() << ", \"qy\": " << q.y() << ", \"qz\": " << q.z() << ", \"qw\": " << q.w() << ",\n"
        << "  \"runtime_s\": " << seconds << ", \"source_points\": " << source_count
        << ", \"source_points_filtered\": " << icp.getPrefilteredReadingPtsCount()
        << ", \"map_points\": " << map_count
        << ", \"map_points_filtered\": " << icp.getPrefilteredReferencePtsCount()
        << ", \"iterations\": " << iterations
        << ", \"iteration_limit_reached\": " << (icp.getMaxNumIterationsReached() ? "true" : "false")
        << ", \"weighted_point_used_ratio\": " << weighted_ratio << "\n}\n";
    std::cout << "ICP " << argv[2] << " -> " << argv[11] << " (" << seconds << " s)\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ICP failed: " << error.what() << "\n";
    return 1;
  }
}
