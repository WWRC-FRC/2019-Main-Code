/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
//import frc.robot.commands.*;
//import frc.robot.*;
//import frc.robot.subsystems.*;
public class CollectCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  boolean eject = false;
  boolean intake = true;
  double distance;
  public CollectCargo(int location) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
     if(location == 1){
     //addSequential(new FindLine());
     addSequential(new LiftToPosition(3,true, 0));
     //addSequential(new DriveToPosition(distance, blocking));
     addSequential(new IntakeManual(intake));
     //addSequential(new LiftToPosition(18,true, 0));
     }
     if(location == 2){
      addSequential(new LiftToPosition(40,true, 0));
      //addSequential(new DriveToPosition(distance, blocking));
      addSequential(new IntakeManual(intake));
     // addSequential(new LiftToPosition(18,true, 0));
     }

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
