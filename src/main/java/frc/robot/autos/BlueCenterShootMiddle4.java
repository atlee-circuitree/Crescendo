// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlwaysRunningIntake;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class BlueCenterShootMiddle4 extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  RobotContainer m_RobotContainer;

  public BlueCenterShootMiddle4(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_RobotContainer = RobotContainer;

    addCommands(

      new ManualShoot(Arm, 120),
      InitialPose("CenterShoot", false),

      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 55),
      ChoreoPathing("CenterShoot", false)
      ),
      new ManualShoot(Arm, 120),

      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 55),
      ChoreoPathing("CenterShoot2", false)
      ),
      new ManualShoot(Arm, 120),
      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 55),
      ChoreoPathing("CenterShoot3", false)
      ),
      m_RobotContainer.LimelightIntake().withTimeout(1.2),
      ChoreoPathing("CenterShoot4", false),
      new ManualShoot(Arm, 120)
      
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

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));
      
    } else {

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).flipped().getInitialPose()));

    }

  }

}
