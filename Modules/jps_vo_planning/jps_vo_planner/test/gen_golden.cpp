// Reference dump for the baseline-identity test T0 of test_fixes (fixes after thesis-frozen-v1).
// Build this file against the thesis-frozen-v1 sources (it uses only their API) and write the reference:
//   gen_golden <out_file>      (default: stdout)
// test_fixes compares its own dump, made with every Fixes flag false, against this file byte for byte.
#include <fstream>
#include <iostream>

#include "golden_scenarios.h"

int main(int argc, char **argv)
{
  const golden::Customize none = [](jps_vo::Switches &) {};
  if (argc > 1)
  {
    std::ofstream f(argv[1]);
    golden::dumpAll(f, none);
    return f.good() ? 0 : 1;
  }
  golden::dumpAll(std::cout, none);
  return 0;
}
