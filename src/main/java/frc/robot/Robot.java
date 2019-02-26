/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.cameraserver.*;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.*;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
//import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.networktables.*;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.DrivesWithJoysticks;
//import edu.wpi.first.wpilibj.VictorSP;
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
//import edu.wpi.first.wpilibj.Timer;



/**
 * TOM IS LIFT,                                                
 * JERRY IS INTAKE,
 * SPIKE IS VISION,
 * NIBBLES IS DRIVE TRAIN,
 * TYKE IS PNEUMATIC SYSTEM,
 * UNCLE PECOS IS ULTRASONIC SENSOR
 */
//https://www.bing.com/videos/search?q=best+tom+and+jerry&view=detail&mid=ADE99A515A0765A73415ADE99A515A0765A73415&FORM=VIRE 
public class Robot extends TimedRobot {
  public static Drive drive = new Drive();
  public static OI m_oi;
  LiftToPosition test;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Command driveWithJoystick;
  Command liftToPosition;
    public static double currentHeightInches = 0.0;
    public static WPI_TalonSRX _rghtFront = new WPI_TalonSRX(7);
    public static WPI_TalonSRX _rghtFollower = new WPI_TalonSRX(9);
    public static WPI_TalonSRX _leftFront = new WPI_TalonSRX(6);
    public static WPI_TalonSRX _leftFollower = new WPI_TalonSRX(11);
    public static WPI_TalonSRX lift = new WPI_TalonSRX(10);
    public static WPI_VictorSPX liftFollower = new WPI_VictorSPX(12);
    public static WPI_TalonSRX intake = new WPI_TalonSRX(8);
    public static final ADIS16470_IMU imu = new ADIS16470_IMU();
    static int countsPerRev = 4096;
    static double wheelSize = 2*Math.PI*(3.0/12.0);
   public static Lift liftSystem = new Lift();
   public static Vision vision = new Vision();
   public static Intake IntakeSystem = new Intake();
   public static UltrasonicSensor ultrasonicSensorVision = new UltrasonicSensor();
   public static UltrasonicSensorDrive ultrasonicSensorDrive = new UltrasonicSensorDrive();
  public static Pneumatics pneumaticsSystem = new Pneumatics();
   public static double encoderFeet = countsPerRev / wheelSize;
  public static DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
  public static Compressor compressor = new Compressor();
    public static Ultrasonic ultrasonicSensor = new Ultrasonic(1, 2, Unit.kMillimeters);
   public static double encoderMeters = encoderFeet*3.281;
   public static DifferentialDrive driveSystem = new DifferentialDrive(_leftFront, _rghtFront);
    public static DigitalInput limitSwitch = new DigitalInput(0);
    public static Hand leftStick = Hand.kLeft;
    public static Hand rightStick = Hand.kRight;
    public static String currentLocation = "Ground";
    
    public static DrivesWithJoysticks driveIntake = new DrivesWithJoysticks();
    //public static boolean;
  //  static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
     _rghtFront.configFactoryDefault();
     _rghtFollower.configFactoryDefault();
     _leftFront.configFactoryDefault();
     _leftFollower.configFactoryDefault();

     //_leftFollower.configReverseSoftLimitEnable(true);

    m_oi = new OI();
    lift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                      Constants.kPIDLoopIdx,
                                      Constants.kTimeoutMs);
    
    lift.setSensorPhase(Constants.kSensorPhase);
    lift.setInverted(Constants.kMotorInvert);

    lift.configNominalOutputForward(0, Constants.kTimeoutMs);
    lift.configNominalOutputReverse(0, Constants.kTimeoutMs);
    lift.configPeakOutputForward(1, Constants.kTimeoutMs);
    lift.configPeakOutputReverse(-.6, Constants.kTimeoutMs);

    lift.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
 
    lift.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		lift.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		lift.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    lift.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		
		/* Set the quadrature (relative) sensor to match absolute */
    lift.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    _rghtFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                      Constants.kPIDLoopIdx,
                                      Constants.kTimeoutMs);
    
    _rghtFront.setSensorPhase(Constants.kSensorPhase);
    _rghtFront.setInverted(Constants.kMotorInvert);

    _rghtFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    _rghtFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _rghtFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    _rghtFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    _rghtFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    _rghtFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_rghtFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_rghtFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _rghtFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		
		/* Set the quadrature (relative) sensor to match absolute */
    _rghtFront.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    _leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                      Constants.kPIDLoopIdx,
                                      Constants.kTimeoutMs);
    
    _leftFront.setSensorPhase(Constants.kSensorPhase);
    _leftFront.setInverted(Constants.kMotorInvert);

    _leftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    _leftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _leftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    _leftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    _leftFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    _leftFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_leftFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_leftFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _leftFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		
		/* Set the quadrature (relative) sensor to match absolute */
    _leftFront.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    

     /* factory default values */
     
   //  imu.calibrate();
      _leftFront.setInverted(true);
      _rghtFront.setInverted(true);
     /* set up followers */
     _rghtFollower.follow(_rghtFront);
     _leftFollower.follow(_leftFront);
     final int kTimeoutMs = 30;
     _rghtFront.configOpenloopRamp(.3, 1000);
     _leftFront.configOpenloopRamp(.3, 1000);
     resetEncoder();
   
     /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
     /*
      * set the invert of the followers to match their respective master controllers
      */


      _rghtFollower.setInverted(InvertType.FollowMaster);
     _leftFollower.setInverted(InvertType.FollowMaster);

     /*
      * [4] adjust sensor phase so sensor moves positive when Talon LEDs are green
      */
     _rghtFront.setSensorPhase(true);
     _leftFront.setSensorPhase(true);

     /*
      * WPI drivetrain classes defaultly assume left and right are opposite. call
      * this so we can apply + to both sides when moving forward. DO NOT CHANGE
      */
     _leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	// Feedback
             6, 											// PID ID
             kTimeoutMs);
     _rghtFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	// Feedback
             7 , 											// PID ID
             kTimeoutMs);
     lift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	// Feedback
             10, 											// PID ID
             kTimeoutMs);
     driveWithJoystick = new DrivesWithJoysticks();
      liftFollower.follow(lift);
    // pneumaticsSmash.setClosedLoopControl(true);
    _leftFront.setNeutralMode(NeutralMode.Brake);
    _rghtFront.setNeutralMode(NeutralMode.Brake);
    _leftFollower.setNeutralMode(NeutralMode.Brake);
    _rghtFollower.setNeutralMode(NeutralMode.Brake);
    compressor.setClosedLoopControl(true);
    
    CameraServer.getInstance().startAutomaticCapture();

    
  }




  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }




  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }




  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }




  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // test = new LiftToPosition(50);

   // solenoid.set(DoubleSolenoid.Value.kForward);

    if(driveWithJoystick != null) driveWithJoystick.start();
   // double visionDrive = SmartDashboard.getNumber("visionDrive", 0.0);
  }




  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
  //  if(OI.getControllerDr().getAButton()){
  //  if(solenoid.get() == Value.kForward){
  //    solenoid.set(Value.kReverse);
  //  if(solenoid.get() == Value.kReverse){
   //   solenoid.set(Value.kForward);
    }
 // }
//}
  // 
    //SmartDashboard.putNumber("Lift Sensor Pos:", lift.getSelectedSensorPosition());
    //lift.set(ControlMode.Position, 10000);
  //}




  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
      if(driveWithJoystick != null) driveWithJoystick.start();
      compressor.setClosedLoopControl(true);
     
  } 




  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
   /*
    String work = "";

        SmartDashboard.putNumber("Right Sensor Vel:", _rghtFront.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Sensor Pos:", -_rghtFront.getSelectedSensorPosition() / encoderMeters);
        SmartDashboard.putNumber("Right Out %",_rghtFront.getMotorOutputPercent());
        SmartDashboard.putBoolean("Right Out Of Phase:", _faults_R.SensorOutOfPhase);
        SmartDashboard.putNumber("Left Sensor Vel:" , _leftFront.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Left Sensor Pos:" , _leftFront.getSelectedSensorPosition() / encoderMeters);
        SmartDashboard.putNumber("Left Out %" , _leftFront.getMotorOutputPercent());
        SmartDashboard.putBoolean("Left Out Of Phase:" , _faults_L.SensorOutOfPhase);
        SmartDashboard.putNumber("Lift Sensor Vel:", lift.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Lift Sensor Pos:", lift.getSelectedSensorPosition()); /// encoderMeters);
        SmartDashboard.putNumber("Lift Out %", lift.getMotorOutputPercent());
        SmartDashboard.putBoolean("Lift Out Of Phase:", _faults_R.SensorOutOfPhase);
        SmartDashboard.putNumber("Angle", imu.getAngle());
        SmartDashboard.putNumber("Conversion Factor", encoderMeters);
        */
        /*
        if(ultrasonicSensorDrive.isUsing) {
          _rghtFront.set(.2);
          _leftFront.set(.2);
        }
        else {
          */
        //  _rghtFront.set(driveIntake.leftSpeed);
         // _leftFront.set(driveIntake.rightSpeed);
        //}
       
  }




  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static void resetEncoder(){
    _leftFront.setSelectedSensorPosition(6,1,1);
    _rghtFront.setSelectedSensorPosition(8,1,1);
    lift.setSelectedSensorPosition(11,1,1);
  }

  public static double leftEncoder(){
    return _leftFront.getSelectedSensorPosition()/encoderMeters;
  }

  public static double rightEncoder(){
    return _rghtFront.getSelectedSensorPosition()/encoderMeters;
  }

  public static double liftEncoder(){
    return lift.getSelectedSensorPosition();//encoderMeters; 38150
  }

  public static double ultrasonicDistance(){
    return ultrasonicSensor.getRangeInches();
  }

  public static boolean isBallIn(){
    return limitSwitch.get();
  }

  public static void read(){
    SmartDashboard.putBoolean("Is ball in?", isBallIn());
    SmartDashboard.putNumber("Lift Sensor Pos:",Robot.lift.getSelectedSensorPosition());
  }

  public static void useUnclePesos(){
    ultrasonicSensorDrive = new UltrasonicSensorDrive();
  }

  public static void setCurrentLocation(double distanceInches){
      if(distanceInches == 25)currentLocation = "Bottom Cargo";
      else if(distanceInches == 53)currentLocation = "Middle Cargo";
      else if(distanceInches == 76)currentLocation = "Top Cargo";
      else if(distanceInches == 3)currentLocation = "Ground";
      else if(distanceInches == 40)currentLocation = "Depot Cargo";
      else if(distanceInches == 38)currentLocation = "Ship Cargo";
      else if(distanceInches == 8)currentLocation = "Bottom Hatch";
      else if(distanceInches == 34)currentLocation = "Middle Hatch";
      else if(distanceInches == 62)currentLocation = "Top Hatch";
      else if(distanceInches == 4)currentLocation = "Hatch Loading Station";
      else currentLocation = "I'm lost";
  }
 
  /*
  public static void driveForward(double distance) {
    _leftFront.setSelectedSensorPosition();
  }
  */
  
 }
