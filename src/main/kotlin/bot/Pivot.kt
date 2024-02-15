package bot

import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.trajectory.TrapezoidProfile

class Pivot: StateSystem<Pivot.Goal, Pivot.State> {
	companion object {
		// <1 for reduction, >1 for increase
		// Pivot is driven by a 15 tooth pulley leading to a 30 tooth pulley
		const val gearing = 15.0/30.0 // unitless, motor -> pivot
		const val posFactor = gearing * (2.0 * Math.PI) // motor rot -> pivot rad
		const val velFactor = posFactor / 60.0 // motor rpm -> pivot rad/s
		const val freeSpeed = 6784.0 * velFactor // pivot rad/s
	}

	private val pivotM = CANSparkFlex(-1, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(20)
		enableVoltageCompensation(12.0)
	}

	private val pivotE = pivotM.getEncoder().apply {
		// TODO: Replace with correct values
		setPositionConversionFactor(posFactor) // motor rotations -> output rad
		setVelocityConversionFactor(velFactor) // motor rpm -> output rad/s
	}

	private val pivotPID = pivotM.getPIDController().apply {
		// TODO: Tune
		// Position PID
		setP(0.1)
		setI(0.0)
		setD(0.0)
		setOutputRange(-1.0, 1.0)
	}

	sealed interface Goal {
		/// Target angle, in radians
		val angle: Double
		/// Target velocity, in rad/s
		val vel: Double
			get() = 0.0

		object Home : Goal { override val angle = 0.0 }
		object Handoff : Goal { override val angle = 0.0 }
		object Amp : Goal { override val angle = 0.0 }
		object Trap : Goal { override val angle = 0.0 }
		object Defense : Goal { override val angle = 0.0 }
		data class Shoot(override val angle: Double) : Goal
	}

	data class State(val angle: Double, val atGoal: Boolean)

	// TODO: Tune
	// Constraints are in rad/s and rad/s^2
	private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(10.0, 10.0))

	override fun applyGoal(goal: Goal): State {
		val setpoint = profile.calculate(0.02, TrapezoidProfile.State(pivotE.getPosition(), pivotE.getVelocity()), TrapezoidProfile.State(goal.angle, goal.vel))
		// TODO: gravity aFF, position offset?
		pivotPID.setReference(setpoint.position, ControlType.kPosition, 0, (setpoint.velocity / freeSpeed) * 12.0, SparkPIDController.ArbFFUnits.kVoltage)
		return State(
			pivotE.getPosition(),
			abs(goal.angle - pivotE.getPosition()) < 0.0 && abs(goal.vel - pivotE.getVelocity()) < 0.0)
	}
}
