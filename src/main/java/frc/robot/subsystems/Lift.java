/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
//import frc.robot.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.Faults;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.analog.adis16470.frc.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.command.Command;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void liftToPosition(double distance) {
    Robot.lift.set(ControlMode.Position, distance);
  }

  @Override
  public void initDefaultCommand() {
  }

  
}
