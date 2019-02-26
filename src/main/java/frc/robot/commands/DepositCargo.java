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
public class DepositCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  boolean eject = false;
  double distance;
  public DepositCargo(double location, boolean blocking) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    
     //addSequential(new FindLine());
     addSequential(new LiftToPosition(location, blocking, 0));
     //addSequential(new DriveToPosition(5, blocking));
     addSequential(new IntakeManual(eject));  
     //addSequential(new DriveToPosition(-5, blocking));

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
