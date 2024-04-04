package bot

import kotlin.math.abs
import kotlin.jvm.optionals.getOrNull
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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.wpilibj.smartdashboard.Field2d
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
	private lateinit var pivot: Pivot
	private lateinit var handoff: Handoff
	private lateinit var climber: Climber
	private lateinit var shotMap: InterpolatingDoubleTreeMap
	private val displayField = Field2d()
	val field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)

	//private lateinit var climbs: List<CANSparkMax>
	override fun robotInit() {
		swerve = Swerve()
		intake = Intake()
		handoff = Handoff()
		pivot = Pivot()
		shooter = Shooter()
		shotMap = InterpolatingDoubleTreeMap()
		shotMap.put(4.24, -0.375)
		shotMap.put(5.495, -0.525)
		shotMap.put(6.62, -0.55)
		shotMap.put(6.323, -0.5375)
		SmartDashboard.putData("Field", displayField)
	 	/*climbs = arrayOf(16, 17).zip(arrayOf(false, false)) { id, inverted -> CANSparkMax(id, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(IdleMode.kBrake)
			setSmartCurrentLimit(80)
			enableVoltageCompensation(12.0)
			setInverted(inverted)
			setPeriodicFramePeriod(PeriodicFrame.kStatus0, 250)
			setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250)
			setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250)
			setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250)
			setPeriodicFramePeriod(PeriodicFrame.kStatus4, 250)
			setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500)
			setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500)
			setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500)
		}}*/

		elevator = Elevator()
		climber = Climber()
		controller = XboxController(0)
		controller2 = XboxController(1)
		controller3 = XboxController(2)
		robotContainer = RobotContainer()
	}
	override fun robotPeriodic() {
		swerve.periodic2()
		displayField.setRobotPose(swerve.getPose())
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
		intake.logPos()
		pivot.dumpPos()
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
		handoff.applyGoal(if (time > 2.5 && time < 4.0) Handoff.Goal.Power(0.6) else Handoff.Goal.Coast)
		//swerve.applyGoal(Swerve.Goal.Drive(Translation2d(if (time > 5.0) -0.4 else 0.0, 0.0), 0.0, true))
		pivot.applyGoal(Pivot.Goal.Home)
	}

	override fun teleopInit() {
		System.out.println("hello!!!!");
		autoCmd?.cancel()
		//swerve.setRotation(swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(180.0)))
	}

	private var lastLL = 0.0
	private var lastLLAbs = Rotation2d()

	override fun disabledPeriodic() {
		LimelightHelpers.setLEDMode_ForceOff("")
	}

	private var allowFeed = false

	override fun teleopPeriodic() {
		SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""))
		SmartDashboard.putNumber("LL TY", LimelightHelpers.getTY(""))
		val blue = DriverStation.getAlliance().getOrNull()?.let { it != Alliance.Red } ?: true
		var joystick = Translation2d(-controller.getLeftY(), -controller.getLeftX())
			.rotateBy(if (blue) Rotation2d() else Rotation2d(Math.PI))
		if (joystick.getNorm() < 0.05) joystick = Translation2d()

		val tag = field.getTagPose(if (blue) 7 else 4).getOrNull()!!.getTranslation().toTranslation2d()
		val pose = swerve.getPose()
		SmartDashboard.putNumber("angle", pose.getRotation().getDegrees())
		val dist = pose.getTranslation().getDistance(tag)

		SmartDashboard.putNumber("distance", dist)

		// 145
		// 190
		var shotAngle = shotMap.get(dist)
		SmartDashboard.putNumber("shot angle", shotMap.get(dist))
		if (shotAngle > -0.375) shotAngle = -0.375
		if (shotAngle < -0.6) shotAngle = -0.6
		val multiplier: Double = if (controller.getLeftBumper()) { 0.5 } else { 1.0 }
		if (controller.getRightBumper()) {
			val transError = tag.minus(pose.getTranslation())
			val targetAngle = transError.getAngle().plus(Rotation2d.fromDegrees(-3.0))
			val error = targetAngle.minus(swerve.getPose().getRotation()).getRadians()
			if (abs(Rotation2d(error).getDegrees()) < 5.0) {
				LimelightHelpers.setLEDMode_ForceOn("")
			} else {
				LimelightHelpers.setLEDMode_ForceOff("")
			}

			var clippedError = error % (2.0 * Math.PI)
			if (clippedError > Math.PI) { clippedError -= 2.0 * Math.PI }
			if (clippedError < -Math.PI) { clippedError += 2.0 * Math.PI }
			val velocity = swerve.getVelocity()
			// trust
			val derivative = (transError.getY() * velocity[0] - transError.getX() * velocity[1]) / (transError.getNorm() * transError.getNorm())
			val pTerm = 1.25 * 9.0 * clippedError
			val dTerm = 0.5 * (derivative - velocity[2])
			val ffTerm = derivative
			swerve.applyGoal(Swerve.Goal.Drive(joystick.times(multiplier * Swerve.maxLinVel), pTerm + dTerm + ffTerm, true))
		} else {
			if (lastLL == 0.0) {
				LimelightHelpers.setLEDMode_ForceOff("")
			} else {
				LimelightHelpers.setLEDMode_PipelineControl("")
			}
			swerve.applyGoal(Swerve.Goal.Drive(joystick.times(multiplier * Swerve.maxLinVel), multiplier * -controller.getRightX() * Swerve.maxAngVel, true))
		}

		var handoffGoal: Handoff.Goal = Handoff.Goal.Coast
		var shooterGoal: Shooter.Goal = Shooter.Goal.Coast
		if (!controller2.getYButton() || !controller2.getLeftBumper()) {
			allowFeed = false
		}
		if (controller2.getRightBumper()) {
			shooterGoal = Shooter.Goal.Other(-0.5)
		}
		if (controller2.getLeftBumper()) {
			shooterGoal = Shooter.Goal.Shoot
		}
		if (controller2.getYButton()) {
			if (shooter.getSpeedPerc() > 0.85) {
				allowFeed = true
			}
			if (!controller2.getLeftBumper() || allowFeed) handoffGoal = Handoff.Goal.Power(1.0)
		}
		if (controller2.getBButton()) {
			handoffGoal = Handoff.Goal.Power(-0.2)
		}

		var elevatorGoal = if (false){ //controller2.getXButton()) {
			Elevator.Goal.Other(-0.1375)
		} else if (controller2.getAButton()) {
			Elevator.Goal.Amp
		} else if (controller2.getLeftBumper() || controller.getRightBumper()) {
			Elevator.Goal.Home // Home
		} else Elevator.Goal.Handoff

		if (controller.getLeftBumper()) {
			//elevatorGoal = Elevator.Goal.Other(-0.015 - 0.0635)
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else if (controller.getYButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Eject))
		} else if (controller2.getXButton()) {
			handoffGoal = Handoff.Goal.Power(0.4)
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Handoff))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Coast))
		}

		elevator.applyGoal(elevatorGoal)

		handoff.applyGoal(handoffGoal)
		shooter.applyGoal(shooterGoal)

		// 6.323
		pivot.applyGoal(if (controller2.getLeftBumper() || controller.getRightBumper()) Pivot.Goal.Shoot(shotAngle) else if (controller2.getAButton() || controller.getXButton()) Pivot.Goal.Home else Pivot.Goal.Amp)

		/*if (controller.getXButton()) {
			climber.applyGoal(Climber.Goal.Pos(110.0))
			//intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else if (controller.getBButton()) {
			climber.applyGoal(Climber.Goal.Pos(-1.5))
		}else {
			climber.applyGoal(Climber.Goal.Coast)
			//intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		}*/

		if (controller.getStartButtonPressed()) {
			swerve.setPose(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
		}
	}

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
