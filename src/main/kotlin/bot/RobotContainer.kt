package bot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

import com.pathplanner.lib.auto.AutoBuilder
class RobotContainer {
	private lateinit var chooser: SendableChooser<Command>

	/// Autos
	init {
		chooser = AutoBuilder.buildAutoChooser()
		chooser.setDefaultOption("Do Nothing", Commands.print("No auto selected"))
		SmartDashboard.putData("Auto", chooser)
	}

	fun getAutonomousCommand(): Command = chooser.getSelected()
}
