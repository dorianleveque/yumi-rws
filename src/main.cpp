#include <ros/ros.h>
#include <abb_librws/rws_interface.h>
#include "test_abb_librws.cpp"

int main(int argc, char** argv)
{
  test_abb_librws::leadThrough();
  //test_abb_librws::retrieveJointPositions();

  std::vector<test_abb_librws::NextPosition> poses;

  //////////////////////////
  // dab pose
  // sync pose
  poses.push_back({
    .leftPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, 135.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });

  // dab pose
  poses.push_back({
    .leftPosition = {-155.0, -77.0, 24.0, 9.0, 12.0, 0.0, 110.0},
    .rightPosition = {21.0, -70.0, -90.0, -190.0, 0.0, 4.0, -12.0},
    .time = 2,
    .waitTime = 0
  });
  // sync pose
  poses.push_back({
    .leftPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, 135.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });
  // end dab pose
  ///////////////////

  /*
  //////////////////////////
  // macarena pose
  // sync pose
  poses.push_back({
    .leftPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, 135.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });

  poses.push_back({
    .leftPosition = {-50.0, -76.0, 18.0, -29.5, 13.5, -26.0, 60.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });

  // p1.1
  poses.push_back({
    .leftPosition = {-150.0, -50.0, -80.0, 136.0, 10.26, -90.0, 70.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });

  poses.push_back({
    .leftPosition = {-150.0, -50.0, -80.0, 136.0, 10.26, -90.0, 70.0},
    .rightPosition = {50.0, -76.0, 30.0, -60.0, 8.0, -80.0, -56.0},
    .time = 2,
    .waitTime = 0
  });

  // p1.2
  poses.push_back({
    .leftPosition = {-150.0, -50.0, -80.0, 136.0, 10.26, -90.0, 70.0},
    .rightPosition = {150.0, -50.0, -80.0, -121.0, 5.35, -102.0, -70.0},
    .time = 2,
    .waitTime = 0
  });

  // p2.1
  poses.push_back({
    .leftPosition = {-150.0, -50.0, -80.0, -48.0, 10.26, -90.0, 70.0},
    .rightPosition = {150.0, -50.0, -80.0, -121.0, 5.35, -102.0, -70.0},
    .time = 1,
    .waitTime = 0
  });

  // p2.2
  poses.push_back({
    .leftPosition = {-150.0, -50.0, -80.0, -48.0, 10.26, -90.0, 70.0},
    .rightPosition = {150.0, -50.0, -80.0, 42.0, 5.35, -102.0, -70.0},
    .time = 1,
    .waitTime = 0
  });

  // macarena bras croisé
  // p3.1
  poses.push_back({
    .leftPosition = {-110.0, -65.0, 50.0, -48.0, 8.30, -87.0, 58.0},
    .rightPosition = {150.0, -50.0, -80.0, 42.0, 5.35, -102.0, -70.0},
    .time = 2,
    .waitTime = 0
  });
  
  // p3.2
  poses.push_back({
    .leftPosition = {-110.0, -65.0, 50.0, -48.0, 8.30, -87.0, 58.0},
    .rightPosition = {110.0, -52.0, 16.0, 12.0, 5.35, -102.0, -70.0},
    .time = 2,
    .waitTime = 0
  });

  // macarena haut de la tête
  // p4.1
  poses.push_back({
    .leftPosition = {-110.0, -65.0, 50.0, -48.0, 8.30, -87.0, 58.0},
    .rightPosition = {58.0, -44.0, 49.0, -86.0, 1.42, -8.0, 45.0},
    .time = 2,
    .waitTime = 0
  });

  // p4.2
  poses.push_back({
    .leftPosition = {-58.0, -48.0, 58.0, -55.0, 8.30, -26.0, -45.0},
    .rightPosition = {58.0, -44.0, 49.0, -86.0, 1.42, -8.0, 45.0},
    .time = 2,
    .waitTime = 0
  });

  // macarena taille
  // p5.1
  poses.push_back({
    .leftPosition = {-58.0, -48.0, 58.0, -55.0, 8.30, -26.0, 130.0},
    .rightPosition = {58.0, -44.0, 49.0, -86.0, 1.42, -8.0, 45.0},
    .time = 2,
    .waitTime = 0
  });
  // p5.2
  poses.push_back({
    .leftPosition = {-58.0, -48.0, 58.0, -55.0, 8.30, -26.0, 130.0},
    .rightPosition = {58.0, -44.0, 49.0, -86.0, 1.42, -8.0, -130.0},
    .time = 2,
    .waitTime = 0
  });
  // sync pose
  poses.push_back({
    .leftPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, 135.0},
    .rightPosition = {0.0, -100.0, 30.0, 0.0, 40.0, 0.0, -135.0},
    .time = 2,
    .waitTime = 0
  });
  // end macarena pose
  ///////////////////
*/

  test_abb_librws::executePoses(poses);
  return 0;
}

