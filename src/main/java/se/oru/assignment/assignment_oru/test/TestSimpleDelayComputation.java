package se.oru.assignment.assignment_oru.test;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterface;


public class TestSimpleDelayComputation {
	
	//FleetMaster Interface Parameters	
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;

	public static void main(String[] args) {
		FleetMasterInterface fleetMasterInterface = new FleetMasterInterface(0., 0., 0., 0.05, 1000, 1000, true, false);
		int robotID1 = 1;
		int robotID2 = 2;
		double maxVelx = 1.0;
		double maxVely = maxVelx * 1;
		fleetMasterInterface.setTrajParams(robotID1, maxVelx, 1);
		fleetMasterInterface.setTrajParams(robotID2, maxVely, 1);
		fleetMasterInterface.setRobotType(robotID1, 1);
		fleetMasterInterface.setRobotType(robotID2, 1);

		Pose ps1 = new Pose(-3, 0, 0);
		Pose ps2 = new Pose(0, -5, Math.PI/2);
		Pose pg1 = new Pose(7, 0.0, 0);
		Pose pg2 = new Pose(0, 5, Math.PI/2);

		fleetMasterInterface.computeTimeDelayWStartandGoal(ps1, pg1, ps2,pg2, robotID1, robotID2);
		//System.out.println("Delay if robot" + robotID1 +  " go first: "+ delays.getFirst());
		//System.out.println("Delay if robot" + robotID2 +  " go first: "+ delays.getSecond());
	}

}
