/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.*;
//import com.ctre.phoenix.motorcontrol.Faults;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.subsystems.*;

public class DriveToPosition extends Command {
  double distanceTicks;
  private boolean blocking;
  private boolean done = false;
  public DriveToPosition(double distanceInches, boolean blocking) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Robot.drive);
    this.blocking = blocking;
    distanceTicks = 512 * distanceInches;
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot._leftFront.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    Robot._rghtFront.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    Robot._leftFront.set(ControlMode.Position, distanceTicks);
    Robot._rghtFront.set(ControlMode.Position, distanceTicks);
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(blocking){
      while((Math.abs(Robot._rghtFront.getSelectedSensorPosition()- distanceTicks)) > 2000){
        
      }
      while((Math.abs(Robot._leftFront.getSelectedSensorPosition()- distanceTicks)) > 2000){}
    }
    done = true;
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
