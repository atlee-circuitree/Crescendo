// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ManualShoot extends Command {
 
  double m_currentVelocity;
  double m_targetVelocity;

  Timer m_spinup;
 
  Arm m_arm;

  public ManualShoot(Arm Arm, double Velocity) {

    m_arm = Arm;
    m_targetVelocity = Velocity;

    m_spinup = new Timer();
 
    addRequirements(Arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_spinup.restart();
 
    m_currentVelocity = m_arm.ReturnVelocity();

    m_arm.RunBottom(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_arm.Spinup(m_targetVelocity);

    if (m_spinup.get() < .2) {

      m_arm.RunBottom(-.03);

    } if (m_spinup.get() > .2 && m_spinup.get() < .8) {
 
      m_arm.Shoot(m_targetVelocity);

    }
   
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.Shoot(0);
    m_arm.Spinup(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_spinup.get() > 1.2) {

      return true;

    } else {

      return false;

    }

  }

}
