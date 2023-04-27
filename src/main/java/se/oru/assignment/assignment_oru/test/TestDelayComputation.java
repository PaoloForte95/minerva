package se.oru.assignment.assignment_oru.test;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterface;



public class TestDelayComputation {
	
	//FleetMaster Interface Parameters	
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;

	public static void main(String[] args) {
		FleetMasterInterface fleetMasterInterface = new FleetMasterInterface(0., 0., 0., 0.05, 1000, 1000, true, false);
		int robotID1 = 1;
		int robotID2 = 2;
		double maxVelx = 4;
		double maxAccx = 1;
		double maxVely = maxVelx * 1;
		double maxAccy = 1;
		fleetMasterInterface.setTrajParams(robotID1, maxVelx, maxAccx);
		fleetMasterInterface.setTrajParams(robotID2, maxVely, maxAccy);
		fleetMasterInterface.setRobotType(robotID1, 1);
		fleetMasterInterface.setRobotType(robotID2, 1);

        Pose startPoseRobot1 = new Pose(18.0,13.0,Math.PI/2);
        Pose startPoseGoal1 = new Pose(22.0,25.0,Math.PI/2);
		Pose goalPoseGoal1 = new Pose(22.0,35.0,Math.PI/2);
        
        Pose startPoseRobot2 = new Pose(21.0,13.0,Math.PI/2);
		Pose startPoseGoal2 = new Pose(19.0,25.0,Math.PI/2);
		Pose goalPoseGoal2 = new Pose(19.0,35.0,Math.PI/2);


        ArrayList<Pose> goals1 = new ArrayList<Pose>();
		goals1.add(startPoseGoal1);
		goals1.add(goalPoseGoal1);

		ArrayList<Pose> goals2 = new ArrayList<Pose>();
		goals2.add(startPoseGoal2);
		goals2.add(goalPoseGoal2);

		PoseSteering[] path1 = fleetMasterInterface.calculatePath(robotID1, startPoseRobot1, goals1, 0.3);
        PoseSteering[] path2 = fleetMasterInterface.calculatePath(robotID2, startPoseRobot2, goals2, 0.3);
        fleetMasterInterface.computeTimeDelay(path1, path2, robotID1, robotID2);

	}

}
