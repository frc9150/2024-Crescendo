package bot

import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import au.grapplerobotics.LaserCan
import au.grapplerobotics.ConfigurationFailedException

class Handoff : StateSystem<Handoff.Goal, Handoff.State> {
	companion object {
		// >1 is gearing increase, <1 is reduction
		const val gearing = 1.0
		// TODO: replace with correct diameter
		const val rollerCirc = Math.PI * (1.3 * .0254) // meters
		const val posFactor = gearing * rollerCirc // rotations -> meters
		const val velFactor = gearing * rollerCirc / 60.0 // rpm -> m/s
		const val freeSpeed = 6784.0 * velFactor // m/s
	}

	// TODO: figure out which motor (if any) needs to be inverted
	private val motor = CANSparkFlex(10, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(40)
		enableVoltageCompensation(12.0)
	}

	private val encoder = motor.getEncoder().apply {
		setPositionConversionFactor(posFactor)
		setVelocityConversionFactor(velFactor)
	}

	/// PID slot 0 is velocity, slot 1 is position
	private val controller = motor.getPIDController().apply {
		// Velocity PID
		setP(1.0, 0)
		setI(0.0, 0)
		setD(0.0, 0)
		setFF(1.0/freeSpeed, 0)
		setOutputRange(-1.0, 1.0, 0)

		// Position PID
		setP(0.1, 1)
		setI(0.0, 1)
		setD(0.0, 1)
		setOutputRange(-1.0, 1.0, 1)
	}

	private val sensor = LaserCan(3).apply {
		try {
		  setRangingMode(LaserCan.RangingMode.SHORT);
		  // position, size
		  // size 4-16
		  setRegionOfInterest(LaserCan.RegionOfInterest(8, 8, 4, 4));
		  setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
		} catch (e: ConfigurationFailedException) {
		  System.out.println("Configuration failed on front LaserCAN! " + e);
		}
	}

	sealed interface Goal {
		sealed interface VelGoal : Goal {
			/// Tangential speed of the shooter wheels, in m/s
			val vel: Double
		}
		/// Brake to hold position/hold note in place
		object Brake : Goal
		/// Allow handoff to free-spin
		object Coast : Goal
		/// Grab from intake
		object Intake : VelGoal { override val vel = 10.0 }
		/// Shoot into speaker
		object Shoot : VelGoal { override val vel = 5.0 }
		/// Deposit into amp
		object Deposit : VelGoal { override val vel = -0.5 }
		/// Custom target velocity
		data class Other(override val vel: Double) : VelGoal
	}

	data class State(val vel: Double, val atGoal: Boolean, val sensor: Double?)

	private var lastGoal: Goal = Goal.Coast
	private var holdPos = 0.0

	override fun applyGoal(goal: Goal): State {
		val atGoal: Boolean

		when (goal) {
			is Goal.Brake -> {
				if (!(lastGoal is Goal.Brake)) {
					holdPos = encoder.getPosition()
				}
				controller.setReference(holdPos, ControlType.kPosition, 1)
				atGoal = true
			}
			is Goal.Coast -> {
				motor.set(0.0)
				atGoal = true
			}
			is Goal.VelGoal -> {
				controller.setReference(goal.vel, ControlType.kVelocity, 0)

				// shooter is considered to be at the correct speed when the velocity is between 90% and 110% of the target velocity
				// TODO: I guess we have to account for goal.vel = 0, smh
				atGoal = abs((encoder.getVelocity() / goal.vel) - 1.0) < 0.1
			}
		}

		val measurement = sensor.getMeasurement()
		val distance = if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
			measurement.distance_mm / 1000.0 
		} else null

		lastGoal = goal
		return State(encoder.getVelocity(), atGoal, distance)
	}

	override fun disable() {
		// Do this so that if commanded to brake after re-enabled, the target position will be reset
		lastGoal = Goal.Coast
	}
}
