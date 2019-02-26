/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeManual extends Command {
  private boolean isSuck;
  private boolean isDone = false; 
  public IntakeManual(boolean isSuck) {
    requires(Robot.IntakeSystem);
    this.isSuck = isSuck;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(isSuck && Robot.isBallIn()){
      Robot.intake.set(.75);
      isDone = false;
    }
    else if(!isSuck){
      Robot.intake.set(-1);
      isDone = false;
    }
    else isDone = true;
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intake.set(.2);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
   Robot.intake.set(.2);
  }
}
