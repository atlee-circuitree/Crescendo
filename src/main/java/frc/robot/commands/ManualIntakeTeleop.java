// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
 
public class ManualIntakeTeleop extends Command {
  /** Creates a new RunIntake. */

  Intake m_intake;
  Arm m_arm;
  double m_velocity;

  public ManualIntakeTeleop(Intake Intake, Arm Arm, double Velocity) {
    
    m_intake = Intake;
    m_arm = Arm;
    m_velocity = Velocity;

    addRequirements(Intake);
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    m_intake.RunIntake(m_velocity);

    if (m_velocity > 0) {

      m_arm.RunBottom(.2);

    } else {

       m_arm.RunBottom(-.1);

    }
 
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    m_intake.RunIntake(0);
    m_arm.RunBottom(0);
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_intake.input.get() == false && m_velocity > 0) {

      return true;

    } else {

      return false;

    }

  }
}
