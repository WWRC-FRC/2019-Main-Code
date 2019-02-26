/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.motorcontrol.ControlMode;
public class HatchRelease extends Command {
  double distanceTicks;
  private double delta;
 // private double oldPosition;
  private boolean localBlocking;
  private double distanceInches;
  private boolean done = false;
  private double currentPosition;
  private int count = 0;
  private String currentLocation;
  public HatchRelease() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
    requires(Robot.liftSystem);
    localBlocking = true;
    currentLocation = Robot.currentLocation;
    System.out.println(currentLocation);
    if(currentLocation.equals("Bottom Hatch")){
    distanceInches = 3;
    distanceTicks = 512 * (distanceInches);
    }
    else if(currentLocation.equals("Middle Hatch")){
      distanceInches = 29;
      distanceTicks = 512 * (distanceInches);
    }
    else if(currentLocation.equals("Top Hatch")){
      distanceInches = 57;
      distanceTicks = 512 * (distanceInches);
    }
    else if(currentLocation.equals("Hatch Loading Station")){
      distanceInches = 8;
      distanceTicks = 512 * (distanceInches);
      }
    else done = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!done){
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
