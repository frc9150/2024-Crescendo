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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame

class Climber : StateSystem<Climber.Goal, Climber.State> {
    val motors = arrayOf(16, 17).zip(arrayOf(false, false)) { id, inverted -> CANSparkMax(id, MotorType.kBrushless).apply {
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
    }}

    val controllers = motors.map { motor -> motor.getPIDController().apply {
		// Position PID
		setP(0.1, 0)
		setI(0.0, 0)
		setD(0.0, 0)
		setOutputRange(-1.0, 1.0, 0)
	}}

    sealed interface Goal {
        data class Pos(val pos: Double) : Goal
        object Coast : Goal
    }
    data class State(val die: Int)

    override fun applyGoal(goal: Goal): State {
        when (goal) {
            is Goal.Pos -> {
                controllers.forEach { it.setReference(goal.pos, ControlType.kPosition) }
            }
            is Goal.Coast -> {
                motors.forEach { it.set(0.0) }
            }
        }
        return State(-100)
    }
}