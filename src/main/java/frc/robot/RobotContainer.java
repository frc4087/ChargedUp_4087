// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;





import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveBase;

import frc.robot.subsystems.ElevatorBase2;



/**-
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 // public AbsoluteEncoder m_encoder;
  
  //private static final SparkMaxAlternateEncoder.Type m_encoder = SparkMaxAlternateEncoder.Type.kQuadrature;
 // public final SparkMaxAlternateEncoder.Type m_encoder = SparkMaxAlternateEncoder.Type.kQuadrature;
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy,1);
  public final JoystickButton bButton = new JoystickButton(opJoy,2);
  public final JoystickButton startButton = new JoystickButton(opJoy,8);
  //public final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
 // public final ProfiledPIDController m_PidController = new ProfiledPIDController(1.3,0.0,0.7, m_Constraints, 0.02);
  public final DriveBase m_DriveBase = new DriveBase();
  public final ElevatorBase2 m_elevatorBase = new ElevatorBase2();
  
  //Users should call reset() when they first start running the controller to avoid unwanted behavior

  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }
  public double getDriveJoyXR() {
    double raw = getDriveJoy(4);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(1);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getOpjoy(int axis){
    double raw = opJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;

  }

  public double getOpjoyYL(){
    double raw = getOpjoy(1);
    return raw;
  }

 


  

  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {




 
   
   //m_ElevatorBase.rightSpark.set(getDriveJoyYL());
    // Configure the trigger bindings
    configureBindings();
  }

  public void robotInit(){
    //SmartDashboard.putNumber("Encoder Value", m_encoder
    //SmartDashboard.putNumber("DriveJoy", getDriveJoyXR());
   



  }

  public void teleopInit(){
     
  

  }

  public void teleopPeriodic(){
    //m_ElevatorBase.leftSpark.set(getDriveJoyXR());
    //m_elevatorBase.leftSpark.set(getOpjoyYL());
   // m_elevatorBase.rightSpark.getAlternateEncoder(Type.kQuadrature, 8192);

  
    m_DriveBase.m_drive.arcadeDrive(getDriveJoyYL(), -getDriveJoyXR());
    
    double setpoint = -20;
    
    //SmartDashboard.putNumber("EncoderL", m_elevatorBase.encoderL.getCountsPerRevolution());
    
    SmartDashboard.putNumber("Talon1",m_DriveBase.leftEncPos);
    SmartDashboard.putNumber("Setpoint",setpoint);
    SmartDashboard.putNumber("Position Error",m_elevatorBase.elevatorPID.getPositionError());
    SmartDashboard.putNumber("EncoderR",m_elevatorBase.encoderR.getPosition());
    SmartDashboard.putNumber("EncoderL",m_elevatorBase.encoderL.getPosition());

    SmartDashboard.putNumber("ElevatorP",Constants.kElevatorP);
    SmartDashboard.putNumber("ElevatorI",Constants.kElevatorI);
    SmartDashboard.putNumber("ElevatorD",Constants.kElevatorD);
    SmartDashboard.putNumber("ElevatorFF",Constants.kElevatorFF);

    //m_elevatorBase.leftSpark.set(getDriveJoyXR());

    // if(opJoy.getBButton()){
    //   m_elevatorBase.leftSpark.set(0.2);
    // }
      
     if(opJoy.getXButton()){
      m_elevatorBase.encoderR.setPosition(0.0);
      m_elevatorBase.encoderL.setPosition(0.0);
     }
      

     if(opJoy.getAButton()){
      m_elevatorBase.rightSpark.set(m_elevatorBase.elevatorPID.calculate(m_elevatorBase.encoderR.getPosition(),setpoint));
      
      //(m_elevatorBase.encoderR.getPosition(),setpoint));
    } 

  

    // if(opJoy.getXButton()){
    //   m_elevatorBase.leftSpark.set(0);
    // }

    //  }else{
    //    m_elevatorBase.leftSpark.set(0);
    //  }
  }


    
  
                                                                                                                                                                                                                             




  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
  
  

