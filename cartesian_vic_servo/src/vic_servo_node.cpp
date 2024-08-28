#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  /*subscribers (real time)
    joint_state
    wrench
    vic_ref (impedance + trajectory)
    (status)
  */

  /*functions
    synchronize joint_state and wrench
    registration (impedance, trajctory in appropriate frame)
    safety (time out)

    update
  */

  /*publisher (real time)
    vic_state
    twist (for moveit)
  */



  printf("hello world cartesian_vic_servo package\n");
  return 0;
}
