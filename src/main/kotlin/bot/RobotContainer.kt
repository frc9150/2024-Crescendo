package bot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class RobotContainer {
	private val chooser = SendableChooser<Command>()

	/// Autos
	init {
		chooser.setDefaultOption("Do Nothing", Commands.print("No auto selected"))
		SmartDashboard.putData("Auto", chooser)
	}

	fun getAutonomousCommand(): Command = chooser.getSelected()
}
