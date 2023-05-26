package se.oru.assignment.assignment_oru.test;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterface;



public class TestDelayComputation {
	
	//FleetMaster Interface Parameters	
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;

	public static void main(String[] args) {
		FleetMasterInterface fleetMasterInterface = new FleetMasterInterface(0., 0., 0., 0.05, 1000, 1000, false, false);
		int robotID1 = 1;
		int robotID2 = 2;
		double maxVelx = 6;
		double maxAccx = 2;
		double maxVely = maxVelx * 1;
		double maxAccy = 2;
		fleetMasterInterface.setTrajParams(robotID1, maxVelx, maxAccx);
		fleetMasterInterface.setTrajParams(robotID2, maxVely, maxAccy);
		fleetMasterInterface.setRobotType(robotID1, 1);
		fleetMasterInterface.setRobotType(robotID2, 1);

        Pose startPoseRobot1 = new Pose(9.0,5.0, 0.0);
        Pose startPoseGoal1 = new Pose(29.0,32.0,Math.PI/2);
		Pose goalPoseGoal1 = new Pose(25.0,52.0,Math.PI/2);
        
        Pose startPoseRobot2 = new Pose(9.0,0.0, 0.0);
		Pose startPoseGoal2 =new Pose(13.0,29.0,Math.PI/2);
		Pose goalPoseGoal2 = new Pose(36.0,52.0,Math.PI/2);

        ArrayList<Pose> goals1 = new ArrayList<Pose>();
		goals1.add(startPoseGoal1);
		goals1.add(goalPoseGoal1);

		ArrayList<Pose> goals2 = new ArrayList<Pose>();
		goals2.add(startPoseGoal2);
		goals2.add(goalPoseGoal2);

		Pair<PoseSteering[], double[]> path1 = fleetMasterInterface.calculatePath(robotID1, startPoseRobot1, goals1, 0.3);
        Pair<PoseSteering[], double[]> path2 = fleetMasterInterface.calculatePath(robotID2, startPoseRobot2, goals2, 0.3);
        fleetMasterInterface.computeTimeDelayWPath(path1.getFirst(), path1.getSecond(), path2.getFirst(), path2.getSecond(), robotID1, robotID2);
		fleetMasterInterface.computeTimeDelay(path1.getFirst()[0].getPose(), goals1, path2.getFirst()[0].getPose(), goals2, robotID1, robotID2);

	}

}
