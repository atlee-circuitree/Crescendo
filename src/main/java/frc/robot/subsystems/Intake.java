// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.Lidar;
 
public class Intake extends SubsystemBase {
 
  TalonFX FrontIntakeMotor;
  TalonFX RearFlyMotor;
  TalonFX FrontFlyMotor;

  VelocityVoltage VelocityVolts;

  private Lidar m_Lidar;
   
  /** Creates a new Intake. */
  public Intake() {
 
    m_Lidar = new Lidar(new DigitalInput(0));
  
    FrontIntakeMotor = new TalonFX(Constants.CAN_IDs.FrontIntakeID, "FRC 1599");
    RearFlyMotor = new TalonFX(Constants.CAN_IDs.RearFlyID, "FRC 1599");
    FrontFlyMotor = new TalonFX(Constants.CAN_IDs.FrontFlyID, "FRC 1599");

    VelocityVolts = new VelocityVoltage(0);

    var VelocityConfig = new Slot0Configs();
    VelocityConfig.kV = 0.12;
    VelocityConfig.kP = 0.11;
    VelocityConfig.kI = 0.48;
    VelocityConfig.kD = 0.01;

    FrontIntakeMotor.getConfigurator().apply(VelocityConfig, 0.050);
 
  }

  @Override
  public void periodic() {
 
    SmartDashboard.putNumber("Lidar", m_Lidar.getDistanceIn());
    
  }

  public void RunIntake(double Velocity) {
 
    FrontIntakeMotor.setControl(VelocityVolts.withVelocity(Velocity));
    FrontFlyMotor.set(Velocity);
    RearFlyMotor.set(Velocity);
 
  }

}