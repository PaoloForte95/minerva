package se.oru.assignment.assignment_oru;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.RobotReport;

public abstract class ComputePathCallback {
	
	public abstract PoseSteering[] computePath(Task task, int pathNumber, RobotReport rr);

}
