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
public class DepositHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  boolean eject = true;
  double distance;
  double downInches;
  public DepositHatch(int location, boolean blocking) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    distance = location;
    downInches = (location-4);
     //addSequential(new FindLine());
     addSequential(new LiftToPosition(distance, true, 0));
     //addSequential(new HatchRelease());
    // addSequential(new HatchRelease());
   //  addSequential(new DriveToPosition(10, true));
   ///  addSequential(new LiftToPosition(downInches,true));
   //  addSequential(new DriveToPosition(-10.0, true));
   //This is a test


    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
