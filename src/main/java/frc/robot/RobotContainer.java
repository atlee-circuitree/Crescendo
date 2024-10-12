// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.BackFeedTest;
import frc.robot.autos.BackfeedBlueCenterShoot4;
import frc.robot.autos.Backfeedexperimental;
import frc.robot.autos.BlueAmp;
import frc.robot.autos.BlueAmpMiddle;
import frc.robot.autos.BlueAmpStealLeft;
import frc.robot.autos.BlueAntiFresta;
import frc.robot.autos.BlueBackUp;
import frc.robot.autos.BlueCenterDelayedBackUp;
import frc.robot.autos.BlueCenterShoot1;
import frc.robot.autos.BlueCenterShootMiddle3;
import frc.robot.autos.BlueCenterShootMiddle4;
import frc.robot.autos.BlueCenterStealMiddle;
import frc.robot.autos.BlueCloseLeftStealLeft;
import frc.robot.autos.BlueCloseLeftStealLeftLineupLeft;
import frc.robot.autos.BlueFourRingLimelight;
import frc.robot.autos.BlueLeftJustMove;
import frc.robot.autos.BlueMiddleSteal;
import frc.robot.autos.BlueRightJustMove;
import frc.robot.autos.BlueRightMoveAndSteal;
import frc.robot.autos.BlueSourceSide;
import frc.robot.autos.CorysTestAuto;
import frc.robot.autos.CorysTestAuto2;
import frc.robot.autos.CorysTestAuto3;
import frc.robot.autos.DriveToSpeaker;
import frc.robot.autos.ExampleAuto;
import frc.robot.autos.LimelightBlueCenterShoot4;
import frc.robot.autos.LimelightFeedTest;
import frc.robot.autos.RedAmp;
import frc.robot.autos.RedAmpMiddle;
import frc.robot.autos.RedAmpVersion2;
import frc.robot.autos.RedAntiFresta;
import frc.robot.autos.RedCenterShoot1;
import frc.robot.autos.RedCenterShootMiddle4;
import frc.robot.autos.RedCenterStealMiddle;
import frc.robot.autos.RedCloseLeft;
import frc.robot.autos.RedCloseLeftStealLeft;
import frc.robot.autos.RedCloseLeftStealLeftLineupLeft;
import frc.robot.autos.RedCorysTestAuto3;
import frc.robot.autos.RedLeftJustMove;
import frc.robot.autos.RedRightJustMove;
import frc.robot.autos.RedSource;
import frc.robot.autos.RedSourceSide;
import frc.robot.autos.SourceLimelighTest;
import frc.robot.autos.TestAmp;
import frc.robot.autos.TestAuto;
import frc.robot.autos.TestSource;
import frc.robot.commands.AutoArm;
import frc.robot.commands.IntakeLights;
import frc.robot.commands.ManualArm;
import frc.robot.commands.ManualHooks;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualIntakeTeleop;
import frc.robot.commands.ManualLeftHook;
import frc.robot.commands.ManualRightHook;
import frc.robot.commands.ManualShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.LimelightHelpers;

public class RobotContainer {
 
  // Variables
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate = 3 * Math.PI; // Half a rotation per second max angular velocity
  SendableChooser<Command> AutoSelect = new SendableChooser<Command>();
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
  private final CommandXboxController Player2 = new CommandXboxController(1);
 
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.DriveTrain;  
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Hooks hooks = new Hooks();
  private final Lights lights = new Lights();
  //private SlewRateLimiter driveLimiter = new SlewRateLimiter(.05);

  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // Basic Drive Command
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * 1)  
        .withVelocityY(Player1.getLeftX() * MaxSpeed * 1)
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    // LEDs
    lights.setDefaultCommand(new IntakeLights(lights, intake));

    // Buttons
    Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), new Rotation2d(135)))));//James changed from Leftbumper 2/24/2024
 
    // Intake and Shoot
    Player1.leftTrigger().whileTrue(new SequentialCommandGroup(
    new AutoArm(arm, 61),                                                                                                             
    new ManualIntakeTeleop(intake, arm, 140)
    ));
    Player1.y().whileTrue(new ManualIntake(intake, arm, -45)); //Rotations per second
    Player1.rightTrigger().whileTrue(new ManualShoot(arm, 500));//120 velocity

    // Limelight Intake
    Player1.leftBumper().whileTrue( 
        new ParallelCommandGroup(
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(2)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14)).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0)));

    Player1.leftBumper().whileTrue(new ManualIntakeTeleop(intake, arm, 120));

    Player1.rightBumper().whileTrue(new AutoArm(arm, Constants.ShootAngle[arm.ConvertedDistance()]));

    // Arm
    Player1.a().onTrue(new AutoArm(arm, 61));
    Player1.b().whileTrue(new ManualArm(arm, 0));
    Player1.x().onTrue(new AutoArm(arm, -50));

    Player1.povDown().whileTrue(new ManualArm(arm, .25));
    Player1.povUp().whileTrue(new ManualArm(arm, -.25));
    
    // Hooks
    Player2.a().whileTrue(new ManualHooks(hooks, 1));
    Player2.y().whileTrue(new ManualHooks(hooks, -1));
    Player2.x().onTrue(new AutoArm(arm, -50));

    Player2.leftBumper().whileTrue(new ManualRightHook(hooks, -.80));
    Player2.leftTrigger().whileTrue(new ManualRightHook(hooks, .80));
    Player2.rightBumper().whileTrue(new ManualLeftHook(hooks, -.80));
    Player2.rightTrigger().whileTrue(new ManualLeftHook(hooks, .80));
 
    // Registers the Telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

    // Auto Options
 
    AutoSelect.setDefaultOption("Blue Center Shoot 4", new BlueCenterShootMiddle4(drivetrain, this, arm, intake));
    AutoSelect.addOption("Red Center Shoot 4", new RedCenterShootMiddle4(drivetrain, this, arm, intake));
    
    //AutoSelect.addOption("Blue LIMELIGHT Shoot 4", new LimelightBlueCenterShoot4(drivetrain, null, arm, intake));
    //AutoSelect.addOption("Blue BACKFEED Shoot 4", new BackfeedBlueCenterShoot4(drivetrain, null, arm, intake));
    //AutoSelect.addOption("Blue Amp Steal Left", new BlueAmpStealLeft(drivetrain, null, intake));
   // AutoSelect.addOption("!!!!", new Backfeedexperimental(drivetrain, null, arm, intake));
    AutoSelect.addOption("Blue Amp", new BlueAmp(drivetrain, this, arm, intake));


    //AutoSelect.addOption("Blue Steal", new TestAuto(drivetrain, this, arm, intake));

    //AutoSelect.addOption("Speaker Steal", new DriveToSpeaker(drivetrain, this, arm, intake));
    //AutoSelect.addOption("New Four Ring", new BlueFourRingLimelight(drivetrain, this, arm, intake));
    AutoSelect.addOption("Blue Source", new TestSource(drivetrain, this, arm, intake));
   // AutoSelect.addOption("Cory test auto", new CorysTestAuto(drivetrain,this,arm,intake));
    //AutoSelect.addOption("Cory test auto 2", new CorysTestAuto2(drivetrain, this, arm, intake));
    //AutoSelect.addOption("Blue Center shoot Steal", new CorysTestAuto3(drivetrain,this,arm,intake));
    AutoSelect.addOption("Red Source Steal", new RedSource(drivetrain,this,arm,intake));
    AutoSelect.addOption("Red Amp", new RedAmp(drivetrain,this,arm,intake));
    //AutoSelect.addOption("Red Center Shoot Steal", new RedCorysTestAuto3(drivetrain, this, arm, intake));
    //AutoSelect.addOption("Red Amp Middle",new RedAmpMiddle(drivetrain, this, arm, intake));
    //AutoSelect.addOption("Blue Amp Middle", new BlueAmpMiddle(drivetrain, this, arm, intake));
    AutoSelect.addOption("Red Amp Version 2", new RedAmpVersion2(drivetrain, this, arm, intake));
    AutoSelect.addOption("Blue Anti Fresta", new BlueAntiFresta(drivetrain,this,arm,intake));
    AutoSelect.addOption("Red Anti Fresta", new RedAntiFresta(drivetrain,this,arm,intake));
    AutoSelect.addOption("Blue New Source Side", new BlueSourceSide(drivetrain,this,arm,intake));
    AutoSelect.addOption("Red New Source Side", new RedSourceSide(drivetrain,this,arm,intake));
    SmartDashboard.putData(AutoSelect);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();
 
  }
 
  public Command getAutonomousCommand() {

    return AutoSelect.getSelected();
    
  }

  public Command LimelightIntake() {

    return new ParallelCommandGroup(

      new ManualIntake(intake, arm, 45),
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(1.5)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))

    );

  }

}
