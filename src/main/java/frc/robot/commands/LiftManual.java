/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.Faults;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.command.Command;


public class LiftManual extends Command {
public boolean isUp;
public boolean isRunning;
  public LiftManual(boolean isUp) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.liftSystem);
    this.isUp = isUp;
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.lift.getSelectedSensorPosition() < 1000 || Robot.lift.getSelectedSensorPosition() > (524*78))isRunning = true;
    if(isUp)Robot.lift.set(ControlMode.PercentOutput, 0.6);
    if(!isUp)Robot.lift.set(ControlMode.PercentOutput, -0.6);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isRunning)return true;
    else return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.lift.set(ControlMode.PercentOutput, 0);
  }
}
