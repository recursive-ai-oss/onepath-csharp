// Black-box driver for RVO2 (C++), written from the public documentation
// (https://gamma.cs.unc.edu/RVO2/documentation/2.0/). Scenario "circle": N agents evenly spread on a circle each
// walk to the antipodal point, no obstacles, fixed time step. Prints bar-format rows.
//   rvo-harness circle --agents N --steps M [--warmup W] [--dt 0.25]
// RVO2 parallelises with OpenMP when it was built with it: set OMP_NUM_THREADS=1 for a single-thread row.
#include <RVO.h>

#include <cmath>
#include <cstdlib>

#include "common.h"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace harness;

int main(int argc, char** argv) {
  Args a(argc, argv);
  if (a.pos.empty() || a.pos[0] != "circle") { std::fprintf(stderr, "usage: rvo-harness circle --agents N --steps M\n"); return 2; }
  const int n = a.integer("agents", 250), steps = a.integer("steps", 200), warm = a.integer("warmup", 20);
  const float dt = (float)a.num("dt", 0.25);
  const std::string scen = a.str("scenario", "circle" + std::to_string(n));
  const float pi = 3.14159265358979f;
  // Radius grows with sqrt(N) so the crowd density stays comparable across sizes (250 agents on radius 200).
  const float R = 200.0f * std::sqrt((float)n / 250.0f);

  RVO::RVOSimulator* sim = new RVO::RVOSimulator();
  sim->setTimeStep(dt);
  sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f);  // the documentation's example defaults
  std::vector<RVO::Vector2> goals;
  for (int i = 0; i < n; ++i) {
    const float ang = 2.0f * pi * (float)i / (float)n;
    sim->addAgent(RVO::Vector2(R * std::cos(ang), R * std::sin(ang)));
    goals.push_back(-sim->getAgentPosition(i));
  }
  auto step = [&]() {
    for (int i = 0; i < n; ++i) {
      RVO::Vector2 d = goals[i] - sim->getAgentPosition(i);
      if (RVO::absSq(d) > 1.0f) d = RVO::normalize(d);
      sim->setAgentPrefVelocity(i, d);
    }
    sim->doStep();
  };
  for (int i = 0; i < warm; ++i) step();
  double t0 = nowMs();
  for (int i = 0; i < steps; ++i) step();
  double ms = nowMs() - t0;

  double sum = 0;  // position checksum: confirms equal work between runs, not a quality claim
  for (int i = 0; i < n; ++i) sum += sim->getAgentPosition(i).x() + 2.0 * sim->getAgentPosition(i).y();
  int threads = 1;
#ifdef _OPENMP
  threads = omp_get_max_threads();
#endif
  const char* side = "rvo2-2.0.2";
  row("crowd", scen, side, "agents", n, "count");
  row("crowd", scen, side, "steps", steps, "count");
  row("crowd", scen, side, "time", ms, "ms");
  row("crowd", scen, side, "agents_per_ms", (double)n * steps / ms, "agent-steps/ms");
  row("crowd", scen, side, "position_checksum", sum, "units");
  row("crowd", scen, side, "threads", threads, "count");
  row("crowd", scen, side, "peak_rss", peakRssMb(), "MB");
  delete sim;
  return 0;
}
