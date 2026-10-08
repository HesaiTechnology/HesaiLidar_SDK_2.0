#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "lidar_types.h"
#include "udp_parser.h"

int main() {
  using namespace hesai::lidar;

  const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto firetimes_path = std::filesystem::temp_directory_path() /
      ("hesai_firetimes_path_test_" + std::to_string(unique_id) + ".csv");

  {
    std::ofstream firetimes_file(firetimes_path);
    if (!firetimes_file) {
      std::cerr << "Could not create temporary firetimes file\n";
      return 1;
    }
    // JT128 skips the header and reads channel,firetime rows.
    firetimes_file << "channel,firetime/us\n1,0.1\n";
  }

  int result = 0;
  {
    UdpParser<LidarPointXYZI> parser("JT128");
    parser.LoadFiretimesFile(firetimes_path.string());
    if (!parser.isSetFiretimeSucc()) {
      std::cerr << "The valid JT128 firetimes file was not loaded\n";
      result = 1;
    } else {
      parser.LoadFiretimesFile("");
      if (!parser.isSetFiretimeSucc()) {
        std::cerr << "An empty firetimes path changed the parser state\n";
        result = 1;
      }
    }
  }

  std::error_code remove_error;
  std::filesystem::remove(firetimes_path, remove_error);
  if (remove_error) {
    std::cerr << "Could not remove temporary firetimes file: " << remove_error.message() << '\n';
    result = 1;
  }

  return result;
}
