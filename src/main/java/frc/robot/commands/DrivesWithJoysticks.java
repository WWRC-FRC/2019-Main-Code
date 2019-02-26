/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.OI;
import frc.robot.Robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
//import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DrivesWithJoysticks extends Command {
 // private double leftSpeed;
  private double left2Speed;
  private double rightSpeed;
  public DrivesWithJoysticks() {
    // Use requires() here to declare subsystem dependencies
   // requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //OI.joyX = OI.getControllerDr().getX(Hand.kLeft);
    //OI.joyY = OI.getControllerDr().getY(Hand.kLeft);

    left2Speed =  OI.getControllerDr().getY(Hand.kLeft); /* positive is forward */
    rightSpeed =  OI.getControllerDr().getY(Hand.kRight); 
    if (Math.abs(left2Speed) < 0.10) 
    {
      left2Speed = 0;
    }
    if (Math.abs(rightSpeed) < 0.10) {
      rightSpeed = 0;
  }
   left2Speed*=.8;
   rightSpeed*=.805;
   if(OI.getControllerDr().getTriggerAxis(Hand.kRight) > 0.05){
    left2Speed*=.75;
    rightSpeed*=.75;
   }
   //System.out.println("Left speed =" + left2Speed);
   //System.out.println("Right speed =" + rightSpeed);
   if(!OI.getControllerDr().getXButton()){
     Robot.driveSystem.tankDrive(left2Speed, rightSpeed);
   }
   else {
     Robot.driveSystem.tankDrive(-.35, -.355);
   }
  /* drive robot */
  }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
