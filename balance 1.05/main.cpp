#include "../../robot/include/main_helper.h"
#include "stand.h"
int main(int argc, char** argv) {
  main_helper(argc, argv, new Leg_InvDyn_Controller());
  return 0;
}
