// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class AutoArm extends Command {
 
  double m_currentAngle;
  double m_targetAngle;
  Arm m_arm;

  public AutoArm(Arm Arm, double Angle) {

    m_arm = Arm;
    m_targetAngle = Angle;

    addRequirements(Arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_currentAngle = m_arm.ReturnCurrentAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_currentAngle = m_arm.ReturnCurrentAngle();

    m_arm.RunAngleWithLimits(Constants.AnglePID.calculate(m_currentAngle, m_targetAngle));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.RunAngleWithLimits(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(Constants.AnglePID.calculate(m_currentAngle, m_targetAngle)) <= 0.05) {

      return true;

    } else {

      return false;

    }

  }

}
