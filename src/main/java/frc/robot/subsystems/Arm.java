// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
 
  TalonFX AngleMotor;
  TalonFX BottomShootingMotor;
  TalonFX CentralShootingMotor;
  TalonFX TopShootingMotor;
 
  DutyCycleEncoder AngleEncoder;

  double CurrentTicks;
  public double CurrentAngle;
 
  VelocityVoltage VelocityVolts;

  int distance;
 
  public Arm() {
 
    // Motors
    AngleMotor = new TalonFX(Constants.CAN_IDs.AngleID,"FRC 1599");
    BottomShootingMotor = new TalonFX(Constants.CAN_IDs.BottomShootingID,"FRC 1599");
    CentralShootingMotor = new TalonFX(Constants.CAN_IDs.CentralShootingID,"FRC 1599");
    TopShootingMotor = new TalonFX(Constants.CAN_IDs.TopShootingID,"FRC 1599");
 
    TopShootingMotor.setNeutralMode(NeutralModeValue.Coast);
    CentralShootingMotor.setNeutralMode(NeutralModeValue.Coast);

    VelocityVolts = new VelocityVoltage(0);

    var VelocityConfig = new Slot0Configs();
    VelocityConfig.kV = 0.12;
    VelocityConfig.kP = 0.11;
    VelocityConfig.kI = 0.48;
    VelocityConfig.kD = 0.01;

    TopShootingMotor.getConfigurator().apply(VelocityConfig, 0.050);
    CentralShootingMotor.getConfigurator().apply(VelocityConfig, 0.050);

    TopShootingMotor.setInverted(true);
    CentralShootingMotor.setInverted(true);

    AngleMotor.setNeutralMode(NeutralModeValue.Brake);

    AngleEncoder = new DutyCycleEncoder(6);

    AngleEncoder.setPositionOffset(.828);

  }

  @Override
  public void periodic() {
 
    /*
    if (AngleEncoder.getAbsolutePosition() < .2) {

      CurrentTicks = AngleEncoder.getAbsolutePosition() + 1;
      
    } else {

      CurrentTicks = AngleEncoder.getAbsolutePosition();

    }
    */

    CurrentTicks = AngleEncoder.getAbsolutePosition();
 
    CurrentAngle = -CurrentTicks / (.072 / 28) + 68;
 
    SmartDashboard.putNumber("Angle Encoder Degrees", CurrentAngle);

    distance = (int) Math.round(getDistanceToAprilTag() * 10);
 
  }

  public double ReturnCurrentAngle() {

    return CurrentAngle;

  }
 
  public void RunAngleWithLimits(double Speed) {

   
    AngleMotor.set(-Speed);
 
 
  }
  
  public void Spinup(double Velocity) {
    
    TopShootingMotor.setControl(VelocityVolts.withVelocity(Velocity));
 
  }

  public void Shoot(double Velocity) {

    TopShootingMotor.setControl(VelocityVolts.withVelocity(Velocity));
    CentralShootingMotor.setControl(VelocityVolts.withVelocity(Velocity));
    if (Velocity == 0) {

      BottomShootingMotor.set(0);

    } else {

      BottomShootingMotor.set(.2);

    }

  }

  public double ReturnVelocity() {

    return ( TopShootingMotor.getVelocity().getValueAsDouble() + 
    CentralShootingMotor.getVelocity().getValueAsDouble() ) / 2;

  }
 
  public void RunBottom(double speed) {

    BottomShootingMotor.set(speed);

  }

  public double getDistanceToAprilTag(){

    double ty = LimelightHelpers.getTY("limelight-sh");
  
    double targetOffsetAngle_Vertical = ty;

    //how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 30;//was30

    //distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 24.5;

    //distance from the low tags to the floor
    double tagHeightInches = 58;
 
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = -1;
 
    distanceFromLimelightToGoalInches = (tagHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
 
    double distanceFromLimelighttoGoalMeters = distanceFromLimelightToGoalInches / 39.37;
 
      return distanceFromLimelighttoGoalMeters;
 
  }

  public int ConvertedDistance() {

    return distance;

  }
    

}