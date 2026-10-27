// Shared helpers for the native black-box drivers: OBJ and point-file loading, timing, row output.
// The drivers print one measurement per line in the bar runner's long format:
//   ROW,<part>,<scenario>,<side>,<metric>,<value>,<unit>
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

namespace harness {

struct Mesh {
  std::vector<float> verts;  // x,y,z triples
  std::vector<int> tris;     // index triples
  int vertCount() const { return static_cast<int>(verts.size() / 3); }
  int triCount() const { return static_cast<int>(tris.size() / 3); }
};

// Minimal Wavefront OBJ reader: "v x y z" and "f a b c ..." (fan-triangulated; "a/b/c" forms accepted).
inline bool loadObj(const std::string& path, Mesh& m) {
  std::ifstream in(path);
  if (!in) return false;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() > 2 && line[0] == 'v' && line[1] == ' ') {
      float x, y, z;
      if (std::sscanf(line.c_str() + 2, "%f %f %f", &x, &y, &z) == 3) {
        m.verts.push_back(x); m.verts.push_back(y); m.verts.push_back(z);
      }
    } else if (line.size() > 2 && line[0] == 'f' && line[1] == ' ') {
      std::istringstream ss(line.substr(2));
      std::string tok;
      std::vector<int> idx;
      while (ss >> tok) idx.push_back(std::atoi(tok.c_str()) - 1);
      for (size_t i = 2; i < idx.size(); ++i) {
        m.tris.push_back(idx[0]); m.tris.push_back(idx[i - 1]); m.tris.push_back(idx[i]);
      }
    }
  }
  return !m.tris.empty();
}

// Query points: one "x y z" per line, '#' comments allowed.
inline std::vector<float> loadPoints(const std::string& path) {
  std::vector<float> p;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    float x, y, z;
    if (std::sscanf(line.c_str(), "%f %f %f", &x, &y, &z) == 3) { p.push_back(x); p.push_back(y); p.push_back(z); }
  }
  return p;
}

inline double nowMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

inline double peakRssMb() {
#if defined(__unix__) || defined(__APPLE__)
  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
#if defined(__APPLE__)
  return static_cast<double>(ru.ru_maxrss) / (1024.0 * 1024.0);  // bytes on macOS
#else
  return static_cast<double>(ru.ru_maxrss) / 1024.0;             // KiB on Linux
#endif
#else
  return 0.0;
#endif
}

struct Args {
  std::map<std::string, std::string> kv;
  std::vector<std::string> pos;
  Args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a.rfind("--", 0) == 0) {
        std::string k = a.substr(2);
        if (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0) kv[k] = argv[++i]; else kv[k] = "1";
      } else pos.push_back(a);
    }
  }
  std::string str(const char* k, const std::string& d = "") const { auto it = kv.find(k); return it == kv.end() ? d : it->second; }
  double num(const char* k, double d) const { auto it = kv.find(k); return it == kv.end() ? d : std::atof(it->second.c_str()); }
  int integer(const char* k, int d) const { auto it = kv.find(k); return it == kv.end() ? d : std::atoi(it->second.c_str()); }
};

inline void row(const char* part, const std::string& scenario, const std::string& side, const char* metric, double value, const char* unit) {
  std::printf("ROW,%s,%s,%s,%s,%.6f,%s\n", part, scenario.c_str(), side.c_str(), metric, value, unit);
}
inline void rowText(const char* part, const std::string& scenario, const std::string& side, const char* metric, const std::string& value) {
  std::printf("ROW,%s,%s,%s,%s,%s,text\n", part, scenario.c_str(), side.c_str(), metric, value.c_str());
}

// Best (minimum) and median of a set of timings.
inline double best(std::vector<double> v) { return *std::min_element(v.begin(), v.end()); }
inline double median(std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; }

}  // namespace harness
