// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hooks extends SubsystemBase {

  //Line
  TalonFX RightHookMotor;
  TalonFX LeftHookMotor;
  PIDController HookPID;
 
  DutyCycleEncoder LeftHookEncoder;
  DutyCycleEncoder RightHookEncoder;
  
  public double RightHookRelative;
 
 
  /** Creates a new Hooks. */
  public Hooks() {
 
    RightHookMotor = new TalonFX(Constants.CAN_IDs.RightHookID,"FRC 1599");
    LeftHookMotor = new TalonFX(Constants.CAN_IDs.LeftHookID,"FRC 1599");

    RightHookMotor.setNeutralMode(NeutralModeValue.Brake);
    LeftHookMotor.setNeutralMode(NeutralModeValue.Brake);

    HookPID = new PIDController(.3, 0, 0);
    
    LeftHookEncoder = new DutyCycleEncoder(1);
    RightHookEncoder = new DutyCycleEncoder(2);
  
 
  }

  public void RunHooks(double speed) {

    RightHookMotor.set(speed);
    LeftHookMotor.set(-speed);

  }

  public void HookPID(double Setpoint) {
 
    RightHookMotor.set(HookPID.calculate(LeftHookEncoder.getAbsolutePosition(), Setpoint));
    LeftHookMotor.set(HookPID.calculate(LeftHookEncoder.getAbsolutePosition(), Setpoint));
 
  }
   
  public void periodic() {
  
    SmartDashboard.putNumber("Left Hook Relative", LeftHookMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Hook Relative", RightHookMotor.getPosition().getValueAsDouble());

  }
  
}
