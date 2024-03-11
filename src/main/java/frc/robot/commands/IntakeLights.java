// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class IntakeLights extends Command {
  /** Creates a new IntakeLights. */
  Lights m_lights;
  Intake m_intake;
 
  public IntakeLights(Lights Lights, Intake Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lights = Lights;
    m_intake = Intake;

    addRequirements(Lights);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_lights.ChangeColorOffRobotState(m_intake.GetSensor());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
