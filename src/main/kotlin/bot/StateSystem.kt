package bot

interface StateSystem<Goal, State> {
	fun applyGoal(goal: Goal): State
	fun disable() {}
}
