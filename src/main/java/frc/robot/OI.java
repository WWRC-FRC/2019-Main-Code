/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.commands.*;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.XboxController;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public int lowCargo = 25;
  public int mediumCargo = 53;
  public int highCargo = 76;
  public int groundCargo = 1;
  public int cargoLoadingStation = 2;
  public int depotCargo = 43;//ToDo : Need to check height
  public int shipCargo = 38;
  public int lowHatch = 8;
  public int mediumHatch = 34;
  public int highHatch = 62;
  public int hatchZone = 4;
  
  //public static double joyX;
  //public static double joyY;

  private Joystick primaryJoystick = new Joystick(0);
  private Joystick secondaryJoystick = new Joystick(1);
  Button togglePneumatics = new JoystickButton(secondaryJoystick, 1);
  Button togglePneumatics2 = new JoystickButton(secondaryJoystick, 2);
  Button LiftPosition1 = new JoystickButton(primaryJoystick,1);
  Button LiftPosition2 = new JoystickButton(primaryJoystick,2);
  Button LiftPosition3 = new JoystickButton(primaryJoystick,3);
  Button LiftPosition4 = new JoystickButton(primaryJoystick,4);
  Button LiftPosition5 = new JoystickButton(primaryJoystick,5);
  Button LiftPosition6 = new JoystickButton(primaryJoystick,6);
  Button LiftPosition7 = new JoystickButton(primaryJoystick,7);
  Button LiftPosition8 = new JoystickButton(primaryJoystick,8);
  Button LiftPosition9 = new JoystickButton(primaryJoystick,9);
  Button LiftPosition10 = new JoystickButton(primaryJoystick,10);
  Button LiftPosition11 = new JoystickButton(primaryJoystick,11);
  Button LiftPosition12 = new JoystickButton(primaryJoystick,12);
  Button LiftPosition13 = new JoystickButton(primaryJoystick,13);
  Button LiftPosition14 = new JoystickButton(primaryJoystick,14);
  Button LiftPosition15 = new JoystickButton(primaryJoystick,15);
  Button LiftPosition16 = new JoystickButton(primaryJoystick,16);
 // Button LiftPosition17 = new JoystickButton(primaryJoystick,17);
 // Button LiftPosition18 = new JoystickButton(primaryJoystick,18);
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
    private static XboxController controllerDr;
  //  private static Button LiftUp;

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create cusLift triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public OI(){
    controllerDr = new XboxController(1);
    
    togglePneumatics.whenPressed(new TogglePneumatics(false));
    togglePneumatics2.whenPressed(new TogglePneumatics(true));
    LiftPosition2.whenPressed(new HatchRelease()); 
    //LiftPosition4.whenPressed(new IntakeToPosition(30.0, false));
    
    LiftPosition4.whenPressed(new BallIn());
    LiftPosition3.whileHeld(new IntakeManual(false));
    LiftPosition6.whileHeld(new IntakeManual(true)); 
    
    //LiftPosition5.whileHeld(new IntakeManual(false));
    LiftPosition10.whileHeld(new DepositCargo(shipCargo, true));
    LiftPosition7.whileHeld(new DepositCargo(lowCargo,true));
    LiftPosition8.whileHeld(new DepositCargo(mediumCargo, true)); 
    LiftPosition9.whileHeld(new DepositCargo(highCargo, true));
    //LiftPosition16.whileHeld(new DepositCargo(shipCargo, false));  
    //LiftPosition15.whileHeld(new DepositCargo(depotCargo, false));  
    
    LiftPosition14.whileHeld(new CollectCargo(groundCargo)); 
    
    LiftPosition11.whileHeld(new DepositHatch(lowHatch, true));
    LiftPosition12.whileHeld(new DepositHatch(mediumHatch, true));
    LiftPosition13.whileHeld(new DepositHatch(highHatch, true));
   // LiftPosition15.whileHeld(new RetrieveHatch(hatchZone);
    LiftPosition15.whenPressed(new DepositHatch(hatchZone, true));
    LiftPosition16.whenPressed(new CollectCargo(cargoLoadingStation));
  }

 public static XboxController getControllerDr() {
    return controllerDr;
  }
}                    