package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAtAprilTag;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.ClimberServo;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TestMotorPair;
import frc.robot.subsystems.Vision;

/* Auto Commands */

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController m_driveController = 
      TuningVariables.useDriveController.getBoolean() ? new XboxController(0) : null;
    private final XboxController m_armController = 
      TuningVariables.useArmController.getBoolean() 
        ? new XboxController(TuningVariables.useDriveController.getBoolean() ? 1 : 0)
        : null;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;


    /* Driver controller Buttons */
    private final JoystickButton zeroGyro = m_driveController != null ? new JoystickButton(m_driveController, XboxController.Button.kY.value) : null;
    private final JoystickButton robotCentric = m_driveController != null ? new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value) : null;
    /* Arm/note handler controller buttons are defined in configureButtonBindings below */

    /* Subsystems */
    private final Swerve s_Swerve = TuningVariables.useSwerve.getBoolean() ? new Swerve() : null; // set s_Swerve to null when testing arm & shooter alone
    /*private final Shoulder m_shoulder = TuningVariables.useShoulder.getBoolean() ? new Shoulder() : null;
    private final Shooter2 m_shooter2 = TuningVariables.useShooter2.getBoolean() ? new Shooter2() : null;
    private final Wrist2 m_wrist2 = TuningVariables.useWrist2.getBoolean() ? new Wrist2() : null;
    private final BeamBreakSensor m_BeamBreakSensor = new BeamBreakSensor();
    private final CollectorRoller m_CollectorRoller = TuningVariables.useCollectorRoller.getBoolean() ? new CollectorRoller() : null;
    private final ClimberServo m_climberServo = new ClimberServo(0);
    private final Vision m_Vision = new Vision("AprilTagCamera");*/
    private final Gyro m_Gyro = new Gyro(false);
    //private final TestMotorPair m_TestMotorPair = new TestMotorPair(CANId.LEFT_ELEVATOR, CANId.RIGHT_ELEVATOR, true);
    //private final Pneumatics m_Pneumatics = new Pneumatics(8, 1);
    private final CoralArm m_CoralArm = new CoralArm(CANId.CORAL_ARM_WRIST, CANId.CORAL_ROLLER, 1);
    private final Elevator m_Elevator = new Elevator(CANId.LEFT_ELEVATOR, CANId.RIGHT_ELEVATOR);
    private final CoralManipulator m_CoralManipulator = new CoralManipulator(m_Elevator, m_CoralArm);
    private final AlgaeArm m_AlgaeArm = new AlgaeArm(CANId.ALGAE_ROLLER);
    //private final SparkMaxMotor m_SparkMaxMotorTest = new SparkMaxMotor(22, 1, "Test motor");
    /* Autos */
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    //private final Command m_Blue1AmpShotAuto = new PathPlannerAuto("Blue1AmpShotAuto");
    //private final Command m_Blue2AmpShotAuto = new PathPlannerAuto("Blue2AmpShotAuto");
        
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        TuningVariables.setAllToDefaultValues();
        if (s_Swerve == null){
            System.out.println("No drivetrain object was created - check TuningVariables in Smartdashboard");
        }
        //if (m_shoulder == null){
            //System.out.println("No shoulder motor objects were created - check TuningVariables in Smartdashboard");
        //}
        if (m_driveController == null){
            System.out.println("No driver's Xbox controller was created, arm controller will be in port 0 - check TuningVariabls in Smartdashboard");
        } else {
            System.out.println("Driver's Xbox controller should be attached to port " + m_driveController.getPort());
        }
        if (m_armController == null) {
            System.out.println("No arm Xbox controller was created - check TurningVariables in Smartdashboard");
        } else {
            System.out.println("Note manipulator's Xbox controller should be attached to port " + m_armController.getPort());
        }
        if (s_Swerve != null) s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -m_driveController.getRawAxis(translationAxis), 
                () -> -m_driveController.getRawAxis(strafeAxis), 
                () -> -m_driveController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean())
            );
            
        //registerNamedPathPlannerCommands();

        //Add commands to the autonomous command chooser
        //m_chooser.setDefaultOption("Leave Starting Zone", new parkAuto(s_Swerve));
        /*m_chooser.addOption("Red Amp And Intake", new redAmpPlusIntakeAuto(s_Swerve, m_shoulder, m_wrist2, m_shooter2, m_CollectorRoller, m_BeamBreakSensor));
        m_chooser.addOption("Blue Amp And Intake", new blueAmpPlusIntakeAuto(s_Swerve, m_shoulder, m_wrist2, m_shooter2, m_CollectorRoller, m_BeamBreakSensor));
        m_chooser.addOption("Center Speaker", new centerSpeaker(s_Swerve, m_shoulder, m_wrist2, m_shooter2));
        m_chooser.addOption("Right Speaker", new rightSpeaker(s_Swerve, m_shoulder, m_wrist2, m_shooter2));
        m_chooser.addOption("Left Speaker", new leftSpeaker(s_Swerve, m_shoulder, m_wrist2, m_shooter2));
        m_chooser.addOption("Center Speaker Plus Center Note", new centerSpeakerScore2(s_Swerve, m_shoulder, m_wrist2, m_shooter2, m_CollectorRoller, m_BeamBreakSensor));
        m_chooser.addOption("Red Speaker + Note Right", null);
        m_chooser.addOption("Blue Speaker + Note Left", null);
        m_chooser.addOption("Do Nothing", new compAuto(s_Swerve, m_shoulder, m_wrist2, m_shooter2, m_CollectorRoller, m_BeamBreakSensor));*/
        //m_chooser.addOption("Amp Shot Auto", m_Blue1AmpShotAuto);
        //m_chooser.addOption("Amp Shot 2 Auto", m_Blue2AmpShotAuto);
        //Put the chooser on the dashboard
        //SmartDashboard.putData(m_chooser);

        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putData("Remove all preferences", new InstantCommand(TuningVariables::removeAllPreferences)); 
        SmartDashboard.putData("Set All TuningVariables to default values", new InstantCommand(TuningVariables::setAllToDefaultValues)); 
        //SmartDashboard.putData(m_Vision);
    }

    //private void registerNamedPathPlannerCommands(){
      //PathPlanner autos use "named commands", which must be registered
      /*Command goToAmpShotPosition = new ParallelCommandGroup(
        new ShoulderGoToPosition(m_shoulder, ShoulderGoToPosition.Method.kRPM, 10.0, -0.185),
        new Wrist2GoToPosition(m_wrist2, 0.06, 0.23)
      ).withTimeout(7.0);
      Command shoot2ForAmp = m_shooter2.shoot2ForAmpCommand();
      Command goToCollectionPosition = new ParallelCommandGroup(
        new ShoulderGoToPosition(m_shoulder, ShoulderGoToPosition.Method.kRPM, 6.0, 0.0),
        new Wrist2GoToPosition(m_wrist2, 0.06, 0)
      ).withTimeout(3.0);
      Command doIntake = m_shooter2.intake2UntilBeamBreak(m_CollectorRoller, m_BeamBreakSensor); */

      //NamedCommands.registerCommand("GoToAmpPosition", goToAmpShotPosition);
      //NamedCommands.registerCommand("Shoot", shoot2ForAmp);
      //NamedCommands.registerCommand("GoToCollectionPosition", goToCollectionPosition);
      //NamedCommands.registerCommand("Collect", doIntake);      
    //}

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        if (s_Swerve != null && zeroGyro != null){
          zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        }
        /* Start assuming the m_armController is not null */
        JoystickButton armA = new JoystickButton(m_armController, XboxController.Button.kA.value);
        JoystickButton armB = new JoystickButton(m_armController, XboxController.Button.kB.value);
        JoystickButton armX = new JoystickButton(m_armController, XboxController.Button.kX.value);
        JoystickButton armY = new JoystickButton(m_armController, XboxController.Button.kY.value);
        JoystickButton armLeftBumper = new JoystickButton(m_armController, XboxController.Button.kLeftBumper.value);
        JoystickButton armRightBumper = new JoystickButton(m_armController, XboxController.Button.kRightBumper.value);
        JoystickButton driveY = new JoystickButton(m_driveController, XboxController.Button.kY.value);
        JoystickButton driveB = new JoystickButton(m_driveController, XboxController.Button.kB.value);
        JoystickButton driveX = new JoystickButton(m_driveController, XboxController.Button.kX.value);
        JoystickButton driveLeftBumper = new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value);
        JoystickButton driveRightBumper = new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value);

        
        //int aprilTagTargetId = 4;

      //Drive buttons:
        
        //driveA.whileTrue(new AimAtAprilTag(m_Vision, s_Swerve, 14, 1, aprilTagTargetId)); //allow 1 degree of error
        driveY.onTrue(new InstantCommand(() -> m_Gyro.setYaw(0.0)));
        driveRightBumper.onTrue(m_AlgaeArm.extendArm());
        driveLeftBumper.onTrue(m_AlgaeArm.retractArm()); //Retracts arm and also sets the roller percent speed to zero
        driveX.whileTrue(m_AlgaeArm.rollerIntake());
        driveX.whileFalse(m_AlgaeArm.rollerHoldAlgae());
        driveB.whileTrue(m_AlgaeArm.rollerPushOut());
        

      //Superstructure buttons:

        //the following all just go to the position and do not actually score it
        armY.onTrue(m_CoralManipulator.goToL4ScoringPosition()); //L4
        armB.onTrue(m_CoralManipulator.goToL3ScoringPosition()); //L3
        armX.onTrue(m_CoralManipulator.goToL2ScoringPosition()); //L2
        armA.onTrue(m_CoralManipulator.goToL1TroughScoringPosition()); //L1

        //other superstructure buttons:
        armRightBumper.onTrue(m_CoralManipulator.pushOutWithRollers());
        armLeftBumper.onTrue(m_CoralManipulator.collectCoral());
        new Trigger(() -> m_armController.getPOV() == 180)
          .onTrue(m_CoralManipulator.dropToScoreOnReef());
        new Trigger(() -> m_armController.getPOV() == 0)
          .onTrue(m_CoralManipulator.goToHome());

    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Translation2d translation2d = new Translation2d(4,0); //TODO: tune this. not too urgent
        // An ExampleCommand will run in autonomous
        return new InstantCommand(() -> s_Swerve.drive(translation2d, 0.0, false, true)); //drive forward
    }
  }