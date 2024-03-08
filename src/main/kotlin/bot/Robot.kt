package bot


import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
//import system.Console

// CAN ids:
// 1,2,3,4: swerve drive br, bl, fl, fr
// 5,6,7,8: swerve angle br, bl, fl, fr
// 9: elevator
// 10: handoff
// 11, 12: shooter (idr which is which)
// Unset, but planned:
// 13: pivot
// 14: intake pivot
// 15: intake motor
// 16, 17: climbers

class Robot : TimedRobot() {
	private var autoCmd: Command? = null
	private var robotContainer: RobotContainer? = null

	private lateinit var intake: Intake
	private lateinit var swerve: Swerve
	private lateinit var elevator: Elevator
	private lateinit var controller: XboxController
	private lateinit var controller2: XboxController
	private lateinit var controller3: XboxController
	private lateinit var shooter: Shooter
	//private lateinit var pivot: Pivot
	private lateinit var handoff: Handoff

	private lateinit var climbs: List<CANSparkMax>
	override fun robotInit() {
		robotContainer = RobotContainer()
		swerve = Swerve()
		intake = Intake()
		handoff = Handoff()
		//pivot = Pivot()
		shooter = Shooter()
	 	climbs = arrayOf(16, 17).zip(arrayOf(true, false)) { id, inverted -> CANSparkMax(id, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(IdleMode.kBrake)
			setSmartCurrentLimit(80)
			enableVoltageCompensation(12.0)
			setInverted(inverted)
		}}

		elevator = Elevator()
		controller = XboxController(0)
		controller2 = XboxController(1)
		controller3 = XboxController(2)
	}
	override fun robotPeriodic() {
		/*val pose = swerve.getPose()
		val transError = Translation2d(0.0, 0.0).minus(pose.getTranslation())
		SmartDashboard.putNumber("swerve x pos", pose.getTranslation().getX())
		SmartDashboard.putNumber("swerve y pos", pose.getTranslation().getY())
		if (controller.getRightBumper() && transError.getNorm() > 0.5) {
			val targetAngle = transError.getAngle()
			val error = targetAngle.minus(pose.getRotation()).getRadians()
			var clippedError = error % (2.0 * Math.PI)
			if (clippedError > Math.PI) { clippedError -= 2.0 * Math.PI }
			if (clippedError < -Math.PI) { clippedError += 2.0 * Math.PI }
			val velocity = swerve.getVelocity()
			// trust
			val derivative = (transError.getY() * velocity[0] - transError.getX() * velocity[1]) / (transError.getNorm() * transError.getNorm())
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-controller.getLeftY() * Swerve.maxLinVel, -controller.getLeftX() * Swerve.maxLinVel), 3.0 * clippedError + derivative, true))
		} else {*/
		CommandScheduler.getInstance().run()
	}
	private var autoStart = 0.0
	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
		autoStart = Timer.getFPGATimestamp()
	}
	override fun autonomousPeriodic() {
		val time = Timer.getFPGATimestamp() - autoStart
		elevator.applyGoal(if (time < 4.0) Elevator.Goal.Handoff else Elevator.Goal.Home)
		shooter.applyGoal(if (time > 1.5 && time < 4.0) Shooter.Goal.Shoot else Shooter.Goal.Coast)
		handoff.applyGoal(if (time > 2.5 && time < 4.0) Handoff.Goal.Power(0.3) else Handoff.Goal.Coast)
		swerve.applyGoal(Swerve.Goal.Drive(Translation2d(if (time > 5.0) -0.4 else 0.0, 0.0), 0.0, true))
	}

	override fun teleopInit() {
		System.out.println("hello!!!!");
		autoCmd?.cancel()
		swerve.setRotation(swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(180.0)))
	}

	override fun teleopPeriodic() {
		val multiplier: Double = if (controller.getLeftBumper()) { 0.5 } else { 1.0 }
		if (controller.getRightBumper()) {
			var targetAngle = Rotation2d.fromDegrees(-60.0)
			val ally = DriverStation.getAlliance()
			if (ally.isPresent() && ally.get() == Alliance.Red) {
				targetAngle = Rotation2d.fromDegrees(60.0)
			}

			val error = targetAngle.minus(swerve.getPose().getRotation()).getRadians()
			var clippedError = error % (2.0 * Math.PI)
			if (clippedError > Math.PI) { clippedError -= 2.0 * Math.PI }
			if (clippedError < -Math.PI) { clippedError += 2.0 * Math.PI }
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(multiplier * -controller.getLeftY() * Swerve.maxLinVel, multiplier * -controller.getLeftX() * Swerve.maxLinVel), 3.5 * clippedError, true))
		} else {
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(multiplier * -controller.getLeftY() * Swerve.maxLinVel, multiplier * -controller.getLeftX() * Swerve.maxLinVel), multiplier * -controller.getRightX() * Swerve.maxAngVel, true))
		}

		var handoffGoal: Handoff.Goal = Handoff.Goal.Coast
		var shooterGoal: Shooter.Goal = Shooter.Goal.Coast
		if (controller2.getRightBumper()) {
			shooterGoal = Shooter.Goal.Other(-0.5)
		}
		if (controller2.getLeftBumper()) {
			shooterGoal = Shooter.Goal.Shoot
		}
		if (controller2.getYButton()) {
			handoffGoal = Handoff.Goal.Power(0.3)
		}
		if (controller2.getBButton()) {
			handoffGoal = Handoff.Goal.Power(-0.2)
		}

		val elevatorGoal = if (controller2.getXButton()) {
			Elevator.Goal.Handoff
		} else if (controller2.getAButton()) {
			Elevator.Goal.Amp
		} else Elevator.Goal.Home
		elevator.applyGoal(elevatorGoal)

		handoff.applyGoal(handoffGoal)
		shooter.applyGoal(shooterGoal)

		intake.logPos()

		if (controller3.getXButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Coast))
		}

		if (controller.getStartButtonPressed()) {
			swerve.setPose(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
		}
	}

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
