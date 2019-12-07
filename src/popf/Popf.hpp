#ifndef POPF_
#define POPF_

#include <string>
#include <vector>

struct PlanItem
{
  float time;
  float duration;
  std::string action;
};

class Popf
{
public:
  Popf() {}

  std::vector<PlanItem> solve(std::string domain_file, std::string problem_file);

};

#endif  // POPF_

