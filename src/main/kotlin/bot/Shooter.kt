package bot

import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType

class Shooter: StateSystem<Shooter.Goal, Shooter.State> {
	companion object {
		// >1 is gearing increase, <1 is reduction
		const val gearing = 1.0
		const val wheelCirc = Math.PI * (3.0 * .0254) // meters
		const val posFactor = gearing * wheelCirc // rotations -> meters
		const val velFactor = gearing * wheelCirc / 60.0 // rpm -> m/s
		const val freeSpeed = 6784.0 * velFactor // m/s
	}

	// TODO: figure out which motor (if any) needs to be inverted
	private val motors = arrayOf(11, 12).zip(arrayOf(true, true)) { id, inverted -> CANSparkFlex(id, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kCoast)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
		setInverted(inverted)
	}}

	private val encoders = motors.map { motor -> motor.getEncoder().apply {
		setPositionConversionFactor(posFactor)
		setVelocityConversionFactor(velFactor)
	}}

	/// PID slot 0 is velocity, slot 1 is position
	private val controllers = motors.map { motor -> motor.getPIDController().apply {
		// Velocity PID
		setP(0.1, 0)
		setI(0.0, 0)
		setD(0.0, 0)
		setFF(1.0/freeSpeed, 0)
		setOutputRange(-1.0, 1.0, 0)

		// Position PID
		setP(0.1, 1)
		setI(0.0, 1)
		setD(0.0, 1)
		setOutputRange(-1.0, 1.0, 1)
	}}

	sealed interface Goal {
		sealed interface Power : Goal {
			val power: Double
		}
		/// Brake to hold position/hold note in place
		object Brake : Goal
		/// Allow shooter to free-spin
		object Coast : Goal
		/// Shoot into speaker
		object Shoot : Power { override val power = 1.0 }
		/// Deposit into amp
		object Deposit : Power { override val power = -0.5 }
		/// Custom target velocity
		data class Other(override val power: Double) : Power
	}

	data class State(val vel: Double, val atGoal: Boolean)

	private var lastGoal: Goal = Goal.Coast
	private var holdPos: List<Double> = listOf()

	override fun applyGoal(goal: Goal): State {
		val atGoal: Boolean

		when (goal) {
			is Goal.Brake -> {
				if (!(lastGoal is Goal.Brake)) {
					holdPos = encoders.map { it.getPosition() }
				}
				controllers.zip(holdPos) { controller, pos -> controller.setReference(pos, ControlType.kPosition, 1) }
				atGoal = true
			}
			is Goal.Coast -> {
				motors.forEach { it.set(0.0) }
				atGoal = true
			}
			is Goal.Power -> {
				motors.forEach { it.set(goal.power) }
				//controllers.forEach { it.setReference(goal.vel, ControlType.kVelocity, 0) }
				atGoal = true /*encoders.all { encoder ->
					// shooter is considered to be at the correct speed when the velocity is between 90% and 110% of the target velocity
					// TODO: I guess we have to account for goal.vel = 0, smh
					abs((encoder.getVelocity() / goal.vel) - 1.0) < 0.1
				}*/
			}
		}

		lastGoal = goal
		return State(encoders.map { it.getVelocity() }.average(), atGoal)
	}

	override fun disable() {
		// Do this so that if commanded to brake after re-enabled, the target position will be reset
		lastGoal = Goal.Coast
	}
}
