// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorBase extends CommandBase {
  public final CANSparkMax leftSpark = new CANSparkMax(14, MotorType.kBrushless);
  public final CANSparkMax rightSpark = new CANSparkMax(15, MotorType.kBrushless);
  public final SparkMaxPIDController elevatorPID = leftSpark.getPIDController();
  //public final RelativeEncoder m_encoderR = rightSpark.getEncoder();
  
  public final RelativeEncoder encoderL = leftSpark.getEncoder();
  public final RelativeEncoder encoderR = rightSpark.getEncoder();


  
  //public final CANSparkMax rightSpark = new CANSparkMax(0,MotorType.kBrushless);
  /** Creates a new ElevatorBase. */
  public ElevatorBase() {

    leftSpark.setSmartCurrentLimit(40);
    rightSpark.setSmartCurrentLimit(40);
  

   // rightSpark.setSmartCurrentLimit(20);
    rightSpark.follow(leftSpark);

    leftSpark.setInverted(true);
    rightSpark.setInverted(false);
  
    elevatorPID.setP(Constants.kElevatorP);
    elevatorPID.setI(Constants.kElevatorI);
    elevatorPID.setD(Constants.kElevatorD);
    elevatorPID.setFF(Constants.kElevatorFF);
    elevatorPID.setIZone(Constants.kElevatorIZ);
    elevatorPID.setOutputRange(0, 0);

    //elevatorPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void setPosition (double setpoint){
    elevatorPID.setReference(setpoint,ControlType.kPosition);
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
