// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //JOYSTICKS -------------------------------------------------------------------------------------------------------------------------------
    
  public static final int XL = 0, //x-axis left joystick
                          YL = 1, //y-axis left joystick
                          XR = 4, //x-acis right joystick
                          YR = 5; //y-axis right joystick

public static final int //buttons
                    kLeftBumper = 5,
                    kRightBumper = 6,
                    kLeftStick = 9,
                    kRightStick = 10,
                    kA = 1,
                    kB = 2,
                    kX = 3,
                    kY = 4,
                    kBack = 7,
                    kStart = 8;

//DRIVEBASE -------------------------------------------------------------------------------------------------------------------------------

//Talons
public final static int //labeled on talons
                    L1=2, 
                    L2=3, 
                    L3=4, 
                    R1=5, 
                    R2=6, 
                    R3=7,
                    R4=9;

//Drive speed limiters
public static final double SpeedDivider = 10.0,
                           secondsForOpenRamp = 0.8,
                           CurrentLimmit = 30;

public final static int kDriveTimeoutMs = 30,
                        kDrivePIDIdx = 0;
//INTAKE ----------------------------------------------------------------------------------------------------------------------------------

//Sparks
public static final int INTAKE = 7; //labeled on spark

//Speeds
public static final double IMSpeed = 1.0; //100% output

//FEEDER ----------------------------------------------------------------------------------------------------------------------------------

//Sparks
public final static int BOTTOMFEED = 10, //labeled on spark
                        TOPFEED = 1; //labeled on spark
      
//Motor Speeds
public static final double  BFMSpeed = 1.0, //100% output
                            TFMSpeed = 1.0; //100% output

//TURRET ----------------------------------------------------------------------------------------------------------------------------------

//Sparks
public final static int TURR = 6; //labeled on spark, well not anymore since we got rid of it

//PID
public final static double  kElevatorP = 0.03,
                            kElevatorI = 0.0,
                            kElevatorD = 0.0,
                            kElevatorFF = 0.0,
                            kElevatorIZ = 0.0;

public final static double turretPosConFac = ((1/70.0)*360); //accounts for gearing

//LAUNCHER --------------------------------------------------------------------------------------------------------------------------------

//Talons
public final static int RLAUNCH = 0, //right front flywheel
      LLAUNCH = 1; //left front flywheel

//Spark
public final static int BLAUNCH = 14; //back flywheel

//Config
public final static int kLaunchTimeoutMs = 30,
      kLaunchPIDIdx = 0;
public final static double backLaunchPosConFac = 1;

 //height limelight is mounted at in feet
     

/**
* Converts the RPM to the speed readable by the talons
*
* @param RPM The speed of the object in rotations per minute
* 
* @return The velocity of that the Talons understands that corresponds to the given RPM
*/
public static double rpmToTalonVel(double RPM){

if(RPM == 0){
return 0;
}
return (RPM/*+500*/)*(2048.0/600.0); //*(24.0/36.0);//look into this
}

/**
* Converts the speed readable by the talons to the RPM
*
* @param vel The speed of the object in rotations per minute
* 
* @return The velocity of that the Talons understands that corresponds to the given RPM
*/
public static double talonVelToRPM(double vel){
return vel / ((2048.0/600.0)); //* (24.0/36.0));
}

//public final static Gains kGains_Vel = new Gains (0.03, 0, 0, 0, 0, 0);//0.3,0.0006,0.2, 0, 0, 0);

public final static double  FrontLauncherHighRPM = 1200, //for high launch
      BackLauncherHighRPM = 1500,
      FrontLauncherLowRPM = 800, //for low launch
      BackLauncherLowRPM = 500; 

//HANGER----------------------------------------------------------------------------------------------------------------------------------



//SENSORS --------------------------------------------------------------------------------------------------------------------------------



//AUTO -----------------------------------------------------------------------------------------------------------------------------------

// Characterization Toolsuite Constants
public static final double ksVolts = 0.205,//0.65634, 
     kvVoltSecondsPerMeter = 2.1,//0.1106, 
     kaVoltSecondsSquaredPerMeter = 0.217,//0.095387,
     kTrackwidthMeters = 0.635,
     kP = 1,//0.17833, 
     kD = 0.0, 
     kMaxSpeedMetersPerSecond = 4.953,
     kMaxAccelerationMetersPerSecondSquared = 8.5344,
     kRamseteB = 2, 
     kRamseteZeta = 0.7;

public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

public static final double kEncoderDistancePerPulse = (4 * Math.PI * 2.54 * 9) / (100.0 * 2048 * 64); //gear ratio is 64 to 9

public static final boolean kGyroReversed = false;
  
}
