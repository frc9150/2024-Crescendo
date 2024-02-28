package bot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase

import com.kauailabs.navx.frc.AHRS

import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.SparkAbsoluteEncoder.Type

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class Swerve : StateSystem<Swerve.Goal, Swerve.State> {
	companion object {
		// TODO: fix
		val trackWidth = Units.inchesToMeters(26.5)
		val wheelBase = Units.inchesToMeters(21.5)

		// Coordinate space?
		val kinematics = SwerveDriveKinematics(
			Translation2d(wheelBase / 2, trackWidth / 2),
			Translation2d(wheelBase / 2, -trackWidth / 2),
			Translation2d(-wheelBase / 2, trackWidth / 2),
			Translation2d(-wheelBase / 2, -trackWidth / 2))

		// TODO
		val maxLinVel = Module.driveFreeSpeed
		val maxModVel = Module.driveFreeSpeed //maxLinVel
		val maxAngVel = 1.0 * Math.PI * 2.0
	}

	sealed interface Goal {
		data class Drive(val linVel: Translation2d, val angVel: Double, val fieldRel: Boolean) : Goal
		data class ModuleStates(val states: Array<SwerveModuleState>) : Goal
		object Lock : Goal
	}

	data class State(val pose: Pose2d)

	// frontLeft, frontRight, backLeft, backRight
	private val modules = arrayOf(
		Module(1, 5, -Math.PI/2),
		Module(2, 6, 0.0),
		Module(4, 8, Math.PI),
		Module(3, 7, Math.PI/2))

	private val gyro = AHRS()//.apply(AHRS::calibrate)

	private val odo = SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getModulePositions())

	private fun drive(linVel: Translation2d, angVel: Double, fieldRel: Boolean) {
		var speeds = ChassisSpeeds(linVel.getX(), linVel.getY(), angVel)
		if (fieldRel) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation())
		val states = kinematics.toSwerveModuleStates(speeds)
		SwerveDriveKinematics.desaturateWheelSpeeds(states, speeds, maxModVel, maxLinVel, maxAngVel)
		SmartDashboard.putNumber("TL Des Speed", states[0].speedMetersPerSecond);
		SmartDashboard.putNumber("Max Speed", maxLinVel);
		modules.zip(states, Module::setDesiredState)
	}

	// Used in auto
	private fun setModuleStates(states: Array<SwerveModuleState>) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, maxModVel)
		modules.zip(states, Module::setDesiredState)
	}

	private fun lockModules() {
		modules.zip(arrayOf(45.0, -45.0, -45.0, 45.0)) { mod, ang -> mod.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(ang))) }
	}

	fun getPose() = odo.getPoseMeters()

	fun setPose(pose: Pose2d) {
		odo.resetPosition(gyro.getRotation2d(), getModulePositions(), pose)
	}

	fun setRotation(rot: Rotation2d) {
		setPose(Pose2d(getPose().getTranslation(), rot))
	}

	fun getModulePositions() = modules.map(Module::getPosition).toTypedArray()

	fun getVelocity(): Array<Double> {
		val chassis = kinematics.toChassisSpeeds(*modules.map(Module::getState).toTypedArray())
		val rotated = Translation2d(chassis.vxMetersPerSecond, chassis.vyMetersPerSecond).rotateBy(getPose().getRotation())
		return arrayOf(rotated.getX(), rotated.getY(), chassis.omegaRadiansPerSecond)
	}

	override fun applyGoal(goal: Goal): State {
		odo.update(gyro.getRotation2d(), getModulePositions())
		when (goal) {
			is Goal.Drive -> {
				drive(goal.linVel, goal.angVel, goal.fieldRel)
			}
			is Goal.ModuleStates -> {
				setModuleStates(goal.states)
			}
			is Goal.Lock -> {
				lockModules()
			}
		}
		return State(getPose())
	}

	fun resetAbsEncoders() = modules.forEach(Module::resetEncoders)

	class Module(driveCanId: Int, turnCanId: Int, val chassisAngularOffset: Double) {
		companion object {
			const val wheelDiameter = 0.0762 // meters
			const val neoFreeSpeedRpm = 6784.0

			const val wheelCircumference = wheelDiameter * Math.PI // meters

			const val drivePinionTeeth = 14.0

			const val driveReduction = (45.0 * 22) / (drivePinionTeeth * 15)

			const val drivePosFac = wheelCircumference / driveReduction // revolutions -> meters
			const val driveVelFac = drivePosFac / 60.0 // rpm -> m/s
			const val driveFreeSpeed = neoFreeSpeedRpm * driveVelFac // m/s

			const val turnPosFac = 2 * Math.PI // revolutions -> radians
			const val turnVelFac = turnPosFac / 60.0 // rpm -> radians per second

			const val turnPIDMinInput = 0.0 // radians
			const val turnPIDMaxInput = turnPosFac // radians
			
			// PID on velocity
			const val driveP = 0.1
			const val driveI = 0.0
			const val driveD = 0.0
			const val driveFF = 1 / driveFreeSpeed

			// PID on position
			// Why?
			const val turnP = 1.0
			const val turnI = 0.0
			const val turnD = 0.0
			const val turnFF = 0.0

			// Why?
			val driveIdle = IdleMode.kBrake
			val turnIdle = IdleMode.kCoast

			const val driveCurrentLim = 50
			const val turnCurrentLim = 20
		}

		val driveM = CANSparkFlex(driveCanId, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(driveIdle)
			setSmartCurrentLimit(driveCurrentLim)
			enableVoltageCompensation(12.0)
		}
		private val turnM = CANSparkMax(turnCanId, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(turnIdle)
			setSmartCurrentLimit(turnCurrentLim)
			enableVoltageCompensation(12.0)
		}

		val driveE = driveM.getEncoder().apply {
			setPositionConversionFactor(drivePosFac)
			setVelocityConversionFactor(driveVelFac)
			setPosition(0.0)
		}
		private val turnE = turnM.getAbsoluteEncoder(Type.kDutyCycle).apply {
			setPositionConversionFactor(turnPosFac)
			setVelocityConversionFactor(turnVelFac)
			setInverted(true)
		}

		private val drivePID = driveM.getPIDController().apply {
			setFeedbackDevice(driveE)
			setP(driveP)
			setI(driveI)
			setD(driveD)
			setFF(driveFF)
			setOutputRange(-1.0, 1.0)
		}
		private val turnPID = turnM.getPIDController().apply {
			setFeedbackDevice(turnE)
			setPositionPIDWrappingEnabled(true)
			setPositionPIDWrappingMinInput(turnPIDMinInput)
			setPositionPIDWrappingMaxInput(turnPIDMaxInput)

			setP(turnP)
			setI(turnI)
			setD(turnD)
			setFF(turnFF)
			setOutputRange(-1.0, 1.0)
		}

		init {
			driveM.burnFlash()
			turnM.burnFlash()
		}

		fun getState() = SwerveModuleState(driveE.getVelocity(), Rotation2d(turnE.getPosition() - chassisAngularOffset))
		fun getPosition() = SwerveModulePosition(driveE.getPosition(), Rotation2d(turnE.getPosition() - chassisAngularOffset))

		fun setDesiredState(state: SwerveModuleState) {
			val corrected = SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(Rotation2d(chassisAngularOffset)))
			val optimized = SwerveModuleState.optimize(corrected, Rotation2d(turnE.getPosition()))
			drivePID.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity)
			turnPID.setReference(optimized.angle.getRadians(), ControlType.kPosition)
		}

		fun resetEncoders() { driveE.setPosition(0.0) }
	}
}
