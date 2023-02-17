// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorBase2 extends SubsystemBase {

  public final PIDController elevatorPID = new PIDController(0.03, 0, 0);
  public final CANSparkMax leftSpark = new CANSparkMax(14, MotorType.kBrushless);
  public final CANSparkMax rightSpark = new CANSparkMax(15, MotorType.kBrushless);
  public final RelativeEncoder encoderL = leftSpark.getEncoder();
  public final RelativeEncoder encoderR = rightSpark.getEncoder();

 // public final SparkMaxPIDController elevatorPID = leftSpark.getPIDController();
  //public final RelativeEncoder m_encoderR = rightSpark.getEncoder();
  
  

  
  
  //public final CANSparkMax rightSpark = new CANSparkMax(0,MotorType.kBrushless);
  /** Creates a new ElevatorBase. */

    

   

    //elevatorPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    // Use addRequirements() here to declare subsystem dependencies.

  /** Creates a new ElevatorBase2. */
  public ElevatorBase2() {
    leftSpark.setSmartCurrentLimit(40);
    rightSpark.setSmartCurrentLimit(40);

    //voltage limits set 2:36 pm 2-17-23
    // leftSpark.setVoltage(-5);
    // rightSpark.setVoltage(-5);
  

    //rightSpark.follow(leftSpark);

    leftSpark.setInverted(true);
    rightSpark.setInverted(false);

    // elevatorPID.setP(Constants.kElevatorP);
    // elevatorPID.setI(Constants.kElevatorI);
    // elevatorPID.setD(Constants.kElevatorD);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
