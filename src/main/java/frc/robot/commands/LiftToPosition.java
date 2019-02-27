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
//import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.subsystems.*;

public class LiftToPosition extends Command {
  double distanceTicks;
  private double delta;
 // private double oldPosition;
  private boolean localBlocking;
  private double distanceInches;
  private boolean done = false;
  private double currentPosition;
  private int count = 0;
  public LiftToPosition(double distanceInches, boolean blocking, double offset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.liftSystem);
    localBlocking = blocking;
    this.distanceInches = distanceInches;
    
    distanceTicks = 512 * (distanceInches + offset);
    Robot.setCurrentLocation(distanceInches);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Another test
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.currentHeightInches = distanceInches;
    //oldPosition = 100000000000.0;
    System.out.println(Robot.currentHeightInches - 5);
    currentPosition = Robot.lift.getSelectedSensorPosition() / 512;
    System.out.println("Start " + currentPosition);
    System.out.println("Requested" + distanceInches);
    Robot.lift.set(ControlMode.Position, distanceTicks);
    System.out.println("blocking =" + localBlocking);
    if(localBlocking){
      System.out.println("inside blocking if");
      while(true)
      {
        delta = (Math.abs(Robot.lift.getSelectedSensorPosition() - distanceTicks));
     //   oldPosition = Robot.lift.getSelectedSensorPosition(); 
        System.out.println("inside loop : " + delta + " " + count);
        if (delta < 2100)break;
        count++;
        
      }
      System.out.println("outside loop");
    }
    System.out.println("done");
    currentPosition = Robot.lift.getSelectedSensorPosition() / 512;
    System.out.println("I am " + currentPosition + " inches from the ground");
    done = true;
    Robot.read();
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
    Robot.lift.set(ControlMode.Position, Robot.lift.getSelectedSensorPosition());
  }

}