// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlwaysRunningIntake;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RedCloseLeftStealLeft extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;

  public RedCloseLeftStealLeft(Drivetrain Drivetrain, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;

    addCommands(

      new ManualShoot(Arm, 70),
      InitialPose("CloseLeft", true),
      new AlwaysRunningIntake(Intake, Arm, 30).withTimeout(.1),
      ChoreoPathing("CloseLeft", true),
      new ManualShoot(Arm, 70),
      ChoreoPathing("StealLeft", true),
      new ManualShoot(Arm, 70)
      
    );

  }

  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    );

  }

  private Command InitialPose(String Trajectory, boolean IsRed) {

    if (IsRed = true) {

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).flipped().getInitialPose()));

    } else {

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));

    }

  }

}
