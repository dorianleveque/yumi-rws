#ifndef TEST_ABB_LIBRWS_H
#define TEST_ABB_LIBRWS_H

#include <ros/ros.h>
#include <abb_librws/rws_interface.h>
#include <abb_librws/rws_common.h>
#include <typeinfo>
namespace test_abb_librws
{
  struct Joint
  {
    float axis1;
    float axis2;
    float axis3;
    float axis4;
    float axis5;
    float axis6;
    float axis7;
  };

  struct NextPosition {
    Joint leftPosition;
    Joint rightPosition;
    double time; // the duration entered specifies the speed of the robot. In other words the total time it will take to get from point A to point B
    double waitTime; // waitting time after move
  };
  
  void leadThrough(void);
  Joint getJointPosition(std::string armName);
  void retrieveJointPositions();
  void executePoses(std::vector<NextPosition> path);

  std::tuple<std::string, std::string> generateRapidCodeControl(std::vector<NextPosition> path);
  bool isMoving(abb::rws::RWSInterface &rws_interface);
  
} // namespace test_abb_librws

#endif
