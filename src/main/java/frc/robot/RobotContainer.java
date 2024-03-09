// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.BlueCenterShoot1;
import frc.robot.autos.BlueCenterShootMiddle4;
import frc.robot.autos.BlueCenterStealMiddle;
import frc.robot.autos.BlueCloseLeft;
import frc.robot.autos.BlueCloseLeftStealLeft;
import frc.robot.autos.BlueCloseLeftStealLeftLineupLeft;
import frc.robot.autos.BlueLeftJustMove;
import frc.robot.autos.BlueRightJustMove;
import frc.robot.autos.BlueRightMoveAndSteal;
import frc.robot.autos.ExampleAuto;
import frc.robot.autos.RedCenterShoot1;
import frc.robot.autos.RedCenterShootMiddle4;
import frc.robot.autos.RedCenterStealMiddle;
import frc.robot.autos.RedCloseLeft;
import frc.robot.autos.RedCloseLeftStealLeft;
import frc.robot.autos.RedCloseLeftStealLeftLineupLeft;
import frc.robot.autos.RedLeftJustMove;
import frc.robot.autos.RedRightJustMove;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualHooks;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;

public class RobotContainer {
 
  // Variables
  private static final double MaxSpeed = 5; // 6 meters per second desired top speed
  private static final double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity
  SendableChooser<Command> AutoSelect = new SendableChooser<Command>();
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
 
  // Subsystems
  private final Drivetrain drivetrain = TunerConstants.DriveTrain;  
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Hooks hooks = new Hooks();

  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // Basic Drive Command
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .85)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .85) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    // Buttons
    Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), new Rotation2d(135)))));//James changed from Leftbumper 2/24/2024
    Player1.a().onTrue(new AutoArm(arm, 68));
    Player1.x().onTrue(new AutoArm(arm, 0));
    Player1.leftTrigger().whileTrue(new ManualIntake(intake, arm, 45)); //Rotations per second
    Player1.rightTrigger().whileTrue(new ManualShoot(arm, 120));
    Player1.leftBumper().whileTrue( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(-1.5)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-sh") / 14)));
    Player1.povUp().whileTrue(new ManualHooks(hooks, .5));
    Player1.povDown().whileTrue(new ManualHooks(hooks, -.5));


    // Registers the Telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

    // Auto Options
    AutoSelect.setDefaultOption("Blue Center Steal", new BlueCenterStealMiddle(drivetrain, arm, intake));
    AutoSelect.addOption("Red Center Steal", new RedCenterStealMiddle(drivetrain, arm, intake));
    AutoSelect.addOption("Blue Center Shoot 4", new BlueCenterShootMiddle4(drivetrain,arm,intake));
    AutoSelect.addOption("Red Center Shoot 4", new RedCenterShootMiddle4(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Close Left, Steal Left, and Lineup Left", new BlueCloseLeftStealLeftLineupLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left, Steal Left, and Lineup Left", new RedCloseLeftStealLeftLineupLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Close Left", new BlueCloseLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left", new RedCloseLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Close Left and Steal Left", new BlueCloseLeftStealLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left and Steal Left", new RedCloseLeftStealLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Center Shoot 1", new BlueCenterShoot1(drivetrain,arm,intake));
    AutoSelect.addOption("Red Center Shoot 1", new RedCenterShoot1(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Right Just Move", new BlueRightJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Red Right Just Move", new RedRightJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Red Left Just Move", new RedLeftJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Left Just Move", new BlueLeftJustMove(drivetrain,arm,intake));
      AutoSelect.addOption("Blue Right Move and Steal", new BlueRightMoveAndSteal(drivetrain,arm,intake));
    SmartDashboard.putData("Select Auto", AutoSelect);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();
 
  }
 
  public Command getAutonomousCommand() {

    return AutoSelect.getSelected();
    
  }

}
