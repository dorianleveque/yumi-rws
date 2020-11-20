#include "test_abb_librws.h"
#include <unistd.h>

namespace test_abb_librws
{
  // ip localhost ou 192.168.125.1
  static std::string ip = "192.168.125.1";

  void leadThrough(void)
  {
    // create interface
    abb::rws::RWSInterface rws_interface(ip);

    ROS_INFO("========== Test Lead-through ==========");
    ROS_INFO_STREAM("connection to the simulator on: " << ip);
    ROS_INFO_STREAM("ROB_L LeadThrough on: " << rws_interface.isLeadThroughOn("ROB_L"));
    ROS_INFO("Press Enter to activate LeadThrough on ROB_L arm");
    std::cin.get();
    rws_interface.setLeadThroughOn("ROB_L");
    ROS_INFO_STREAM("ROB_L LeadThrough on: " << rws_interface.isLeadThroughOn("ROB_L"));

    ROS_INFO("Press Enter to deactivate LeadThrough on ROB_L arm");
    std::cin.get();
    rws_interface.setLeadThroughOff("ROB_L");
    ROS_INFO_STREAM("ROB_L LeadThrough on: " << rws_interface.isLeadThroughOn("ROB_L"));
  }

  Joint getJointPosition(std::string armName)
  {
    // create interface
    abb::rws::RWSInterface rws_interface(ip);
    
    // Read current jointtarget.
    abb::rws::JointTarget c;
    rws_interface.getMechanicalUnitJointTarget(armName, &c);

    return (Joint){
        .axis1 = (float)c.robax.rax_1.value,
        .axis2 = (float)c.robax.rax_2.value,
        .axis3 = (float)c.robax.rax_3.value,
        .axis4 = (float)c.robax.rax_4.value,
        .axis5 = (float)c.robax.rax_5.value,
        .axis6 = (float)c.robax.rax_6.value,
        .axis7 = (float)c.extax.eax_a.value};
  }

  void retrieveJointPositions()
  {
    while (true)
    {
      Joint j = getJointPosition("ROB_R");
      printf("j1:%f, j2:%f, j3:%f, j4:%f, j5:%f, j6:%f, j7:%f\n", j.axis1, j.axis2, j.axis3, j.axis4, j.axis5, j.axis6, j.axis7);
      usleep(1000 * 500);
    }
  }

  std::tuple<std::string, std::string> generateRapidCodeControl(std::vector<NextPosition> path)
  {
    std::string leftModuleContent = "MODULE LeftArmControl\n";
    std::string rightModuleContent = "MODULE RightArmControl\n";
    for (int i=0; i<path.size(); i++)
    {
      Joint l = path[i].leftPosition;
      Joint r = path[i].rightPosition;
      leftModuleContent += "  VAR jointtarget p"+std::to_string(i)+":= [["+std::to_string(l.axis1)+','+std::to_string(l.axis2)+','+std::to_string(l.axis3)+','+std::to_string(l.axis4)+','+std::to_string(l.axis5)+','+std::to_string(l.axis6)+"], ["+std::to_string(l.axis7)+",0,0,0,0,0]];\n";
      rightModuleContent += "  VAR jointtarget p"+std::to_string(i)+":= [["+std::to_string(r.axis1)+','+std::to_string(r.axis2)+','+std::to_string(r.axis3)+','+std::to_string(r.axis4)+','+std::to_string(r.axis5)+','+std::to_string(r.axis6)+"], ["+std::to_string(r.axis7)+",0,0,0,0,0]];\n";
    }
    leftModuleContent += "  VAR bool moving := true;\n";
    leftModuleContent += "  PROC main()\n";
    rightModuleContent += "  VAR bool moving := true;\n";
    rightModuleContent += "  PROC main()\n";
    for (int i=0; i<path.size(); i++)
    {
      leftModuleContent += "    MoveAbsJ p"+ std::to_string(i) +", v1000,\\T:="+ std::to_string(path[i].time) +", fine,tool0;\n";
      leftModuleContent += "    WaitTime "+ std::to_string(path[i].waitTime) +";\n";
      rightModuleContent += "    MoveAbsJ p"+ std::to_string(i) +", v1000,\\T:="+ std::to_string(path[i].time) +", fine,tool0;\n";
      rightModuleContent += "    WaitTime "+ std::to_string(path[i].waitTime) +";\n";
    }
    leftModuleContent += "    moving := false;\n";
    leftModuleContent += "    Stop;\n";
    leftModuleContent += "  ENDPROC\n";
    leftModuleContent += "ENDMODULE\n";
    rightModuleContent += "    moving := false;\n";
    rightModuleContent += "    Stop;\n";
    rightModuleContent += "  ENDPROC\n";
    rightModuleContent += "ENDMODULE\n";

    return std::make_tuple(leftModuleContent, rightModuleContent);
  }

  bool isMoving(abb::rws::RWSInterface &rws_interface)
  {
    abb::rws::RAPIDBool leftMoving;
    abb::rws::RAPIDBool rightMoving;
    rws_interface.getRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_L, "LeftArmControl", "moving", &leftMoving);
    rws_interface.getRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_R, "RightArmControl", "moving", &rightMoving);
    return (leftMoving.value == true || rightMoving.value == true);
  }

  void executePoses(std::vector<NextPosition> path)
  {
    // create interface
    abb::rws::RWSInterface rws_interface(ip);

    // stop RAPID code before loading new RAPID program
    rws_interface.stopRAPIDExecution();
    while(rws_interface.isRAPIDRunning()==true) {} // wait 

    // launch RAPID code generation for each arm
    std::string leftModuleContent, rightModuleContent;
    tie(leftModuleContent, rightModuleContent) = generateRapidCodeControl(path);

    // file creation
    abb::rws::RWSClient::FileResource programFile("ControlProgram.pgf", abb::rws::SystemConstants::RWS::Identifiers::HOME_DIRECTORY + "/YumiProgram");
    abb::rws::RWSClient::FileResource moduleLeftArmControlFile("LeftArmControl.mod", abb::rws::SystemConstants::RWS::Identifiers::HOME_DIRECTORY + "/YumiProgram/modules");
    abb::rws::RWSClient::FileResource moduleRightArmControlFile("RightArmControl.mod", abb::rws::SystemConstants::RWS::Identifiers::HOME_DIRECTORY + "/YumiProgram/modules");

    // send RAPID program and module to YuMI
    // /!\ Check before uploading that the folder is created
    rws_interface.uploadFile(programFile, "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n<Program>\n</Program>\n");
    rws_interface.uploadFile(moduleLeftArmControlFile, leftModuleContent);
    rws_interface.uploadFile(moduleRightArmControlFile, rightModuleContent);

    // load RAPID program in each arm
    // waiting times are necessary, if it is too fast, the program or module will not be taken into account
    std::cout << "Loading programs\n";
    rws_interface.loadProgramIntoTask(abb::rws::SystemConstants::RAPID::TASK_ROB_L, programFile);
    usleep(1000 * 100);
    rws_interface.loadProgramIntoTask(abb::rws::SystemConstants::RAPID::TASK_ROB_R, programFile);
    std::cout << "Loading modules\n";
    
    // load the modules in the program of each arm
    usleep(1000 * 100);
    rws_interface.loadModuleIntoTask(abb::rws::SystemConstants::RAPID::TASK_ROB_L, moduleLeftArmControlFile);
    usleep(1000 * 100);
    rws_interface.loadModuleIntoTask(abb::rws::SystemConstants::RAPID::TASK_ROB_R, moduleRightArmControlFile);
    usleep(1000 * 500);


    std::cout << "program loaded\n";
    rws_interface.resetRAPIDProgramPointer();
    std::cout << "reset pointer\n";
    rws_interface.startRAPIDExecution();
    while(rws_interface.isRAPIDRunning()==false) {} // wait
    
    // wait end moving
    while(isMoving(rws_interface)) {}
  }

  void pose()
  {
    // /!\ before executing, make sure you have loaded the RAPID programs from the RWS/src/rapid folder into the robot

    // create interface
    abb::rws::RWSInterface rws_interface(ip);
    // describe different position
    abb::rws::JointTarget syncPositionLeft;
    syncPositionLeft.robax.rax_1.value = 0.0;
    syncPositionLeft.robax.rax_2.value = -130.0;
    syncPositionLeft.robax.rax_3.value = 30.0;
    syncPositionLeft.robax.rax_4.value = 0.0;
    syncPositionLeft.robax.rax_5.value = 40.0;
    syncPositionLeft.robax.rax_6.value = 0.0;
    syncPositionLeft.extax.eax_a.value = 135.0;

    abb::rws::JointTarget syncPositionRight;
    syncPositionRight.robax.rax_1.value = 0.0;
    syncPositionRight.robax.rax_2.value = -130.0;
    syncPositionRight.robax.rax_3.value = 30.0;
    syncPositionRight.robax.rax_4.value = 0.0;
    syncPositionRight.robax.rax_5.value = 40.0;
    syncPositionRight.robax.rax_6.value = 0.0;
    syncPositionRight.extax.eax_a.value = -135.0;

    abb::rws::JointTarget dabPositionLeft;
    dabPositionLeft.robax.rax_1.value = -155.0;
    dabPositionLeft.robax.rax_2.value = -77.0;
    dabPositionLeft.robax.rax_3.value = 24.0;
    dabPositionLeft.robax.rax_4.value = 9.0;
    dabPositionLeft.robax.rax_5.value = 12.0;
    dabPositionLeft.robax.rax_6.value = 0.0;
    dabPositionLeft.extax.eax_a.value = 110.0;

    abb::rws::JointTarget dabPositionRight;
    dabPositionRight.robax.rax_1.value = 21.0;
    dabPositionRight.robax.rax_2.value = -70.0;
    dabPositionRight.robax.rax_3.value = -90.0;
    dabPositionRight.robax.rax_4.value = -190.0;
    dabPositionRight.robax.rax_5.value = 0.0;
    dabPositionRight.robax.rax_6.value = 4.0;
    dabPositionRight.extax.eax_a.value = -12.0;

    abb::rws::RAPIDNum speed;

    rws_interface.resetRAPIDProgramPointer();
    rws_interface.startRAPIDExecution();

    ROS_INFO("1: Press Enter to start dab pose");
    std::cin.get();
    speed.value = 100.0;
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_L, "ProgramPoseL", "speed", speed);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_R, "ProgramPoseR", "speed", speed);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_L, "ProgramPoseL", "position", dabPositionLeft);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_R, "ProgramPoseR", "position", dabPositionRight);

    ROS_INFO("2: Press Enter to start sync pose");
    std::cin.get();
    speed.value = 50.0;
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_L, "ProgramPoseL", "speed", speed);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_R, "ProgramPoseR", "speed", speed);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_L, "ProgramPoseL", "position", syncPositionLeft);
    rws_interface.setRAPIDSymbolData(abb::rws::SystemConstants::RAPID::TASK_ROB_R, "ProgramPoseR", "position", syncPositionRight);
  }

} // namespace test_abb_librws