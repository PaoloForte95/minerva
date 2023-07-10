package se.oru.assignment.assignment_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;


import com.sun.jna.Native;
import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;

import aima.core.util.datastructure.Pair;

import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterfaceLib.EnvelopeGridParams;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterfaceLib.TrajParams;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterfaceLib.VehicleModel2d;


public abstract class AbstractFleetMasterInterface {
	protected HashMap<Integer, Long> paths = null; //teID (or this pathID), fleetmaster pathID 
	protected HashMap<Integer, TrajParams> trajParams = null; //robotID, trajectory parameters
	protected EnvelopeGridParams gridParams = null;
	//protected VehicleModelParam vehicleParam = null;
	protected PointerByReference p = null;
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(FleetMasterInterface.class);
	
	/**
	 * The default footprint used for robots if none is specified.
	 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	 */
	public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};
	
	/**
	 * The default trajectory params used for robots if none is specified.
	 * (Values are defined according to the file trajectory_processor.h).
	 */
	public static TrajParams DEFAULT_TRAJ_PARAMS = null;


	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		//If the library is located in a custom location, use the line below
		//NativeLibrary.addSearchPath("eclnav_fleet", "FleetMasterInterface");
		INSTANCE = Native.load("eclnav_assignment", FleetMasterInterfaceLib.class);
	}
	
	/**
	 * Class constructor.
	 * @param origin_x The x coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta origin of the lower-left pixel map (counterclockwise rotation. 
	 * 					   However, many parts of the system currently ignore it).
	 * @param resolution The resolution of the map (in meters/cell). It is assumed this parameter to be global among the fleet.
	 * @param width Number of columns of the map (in cells) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (in cells) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public AbstractFleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean debug) {

		gridParams = new EnvelopeGridParams();
		gridParams.origin.x = origin_x;
		gridParams.origin.y = origin_y;
		gridParams.origin.theta = origin_theta;
		gridParams.resolution = resolution;
		gridParams.width = new NativeLong(width);
		gridParams.height = new NativeLong(height);
		gridParams.dynamic_size = dynamic_size;
		gridParams.debug = debug;
		gridParams.robotID = 1;
		this.paths = new HashMap<Integer, Long>();
		this.trajParams = new HashMap<Integer, TrajParams>();
	};
	
	/**
	 * Abstract class constructor
	 */
	public AbstractFleetMasterInterface() {
		this(0., 0., 0., 0.05, 1000, 1000, true, false);
	}
	
	/**
	 * Initialize parameters.
	 */
	public boolean init() {
		if (gridParams == null) return false;
		p = INSTANCE.init(gridParams);
		return true;
	}

	/**
	 * Set the value of the footprint used for robots if none is specified.
	 */
	public boolean setDefaultFootprint(Coordinate ... coordinates) {
		if (coordinates.length == 0) return false;
		DEFAULT_FOOTPRINT = coordinates;
		return true;
	}
	
	/**
	 * Return the value of the trajectory parameters used for robots if none is specified.
	 */
	public Coordinate[] getDefaultFootprint() {
		return DEFAULT_FOOTPRINT;
	}
	
	
	/**
	 * Set the trajectory parameters for a robot. Use <code>-1</code> when the parameter is not known,
	 * where maxSteeringAngleVel equal to <code>-1</code> will automatically force useSteerDriveVel to be <code>false</code>.
	 * @param robotID The robot ID.
	 */
	public void setTrajParams(int robotID, TrajParams param) {;
		
		trajParams.put(robotID, param);
		INSTANCE.setTrajectoryParams(p, robotID, param, param.debugPrefix);
	}

	/**
	 * Set the trajectory parameters for a robot. Use <code>-1</code> when the parameter is not known,
	 * where maxSteeringAngleVel equal to <code>-1</code> will automatically force useSteerDriveVel to be <code>false</code>.
	 * @param robotID The robot ID.
	 */
	public void setTrajParams(int robotID, double maxVel, double maxAcc) {;
		TrajParams param = new TrajParams();
		param.maxVel = maxVel;
		trajParams.put(robotID, param);
		INSTANCE.setTrajectoryParams(p, robotID, param, param.debugPrefix);
	}


	public void setRobotType(int robotID, int vehicle_type){
		VehicleModel2d robotType = new VehicleModel2d();
		robotType.type = vehicle_type;
		metaCSPLogger.info("Setting type vehicle " + robotID + " to " + robotType.type);
		INSTANCE.setVehicleType(p, robotID, robotType);
	}


	public Pair<Double,Double> computeTimeDelayWPath(PoseSteering[] path1, double[] curvs1, PoseSteering[] path2, double[] curvs2, int robotID1, int robotID2 ){
		PathPose[] pi1 = (PathPose[])new PathPose().toArray(path1.length);
		double[] steering = new double[path1.length];
		for (int i = 0; i < path1.length; i++) {
			pi1[i].x = path1[i].getX();
			pi1[i].y = path1[i].getY();
			pi1[i].theta = path1[i].getTheta();
			steering[i] = path1[i].getSteering();
		}

		PathPose[] pi2 = (PathPose[])new PathPose().toArray(path2.length);
		double[] steering2 = new double[path2.length];
		for (int i = 0; i < path2.length; i++) {
			pi2[i].x = path2[i].getX();
			pi2[i].y = path2[i].getY();
			pi2[i].theta = path2[i].getTheta();
			steering2[i] = path2[i].getSteering();
		}

		DoubleByReference delayTe1 = new DoubleByReference();
		DoubleByReference delayTe2 = new DoubleByReference();
		INSTANCE.computeTimeDelayWPath(p, pi1,  path1.length, curvs1, pi2, path2.length, curvs2, robotID1, robotID2, delayTe1,delayTe2);
		return new Pair<Double, Double>(delayTe1.getValue(), delayTe2.getValue());
	}

	public Pair<Double,Double> computeTimeDelayWPath(PoseSteering[] path1, PoseSteering[] path2, int robotID1, int robotID2 ){
		double[] curvs1 = new double[path1.length];
		double[] curvs2 = new double[path2.length];

		Arrays.fill(curvs1, 0.0);
		Arrays.fill(curvs2, 0.0);

		return computeTimeDelayWPath(path1,curvs1,path2,curvs2,robotID1, robotID2);
	}


	public Pair<Double,Double> computeTimeDelay(Pose start1, ArrayList<Pose> goals1, Pose start2, ArrayList<Pose> goals2, int robotID1, int robotID2 ){
		
		PathPose ps1 = new PathPose();
		ps1.x = start1.getX();
		ps1.y = start1.getY();
		ps1.theta = start1.getTheta();
		PathPose[] pgs1 = (PathPose[])new PathPose().toArray(goals1.size());
		for (int i = 0; i < goals1.size(); i++) {
			pgs1[i].x = goals1.get(i).getX();
			pgs1[i].y = goals1.get(i).getY();
			pgs1[i].theta = goals1.get(i).getTheta();
		}

		PathPose ps2 = new PathPose();
		PathPose[] pgs2 = (PathPose[])new PathPose().toArray(goals2.size());
		ps2.x = start2.getX();
		ps2.y = start2.getY();
		ps2.theta = start2.getTheta();
		for (int i = 0; i < goals2.size(); i++) {
			pgs2[i].x = goals2.get(i).getX();
			pgs2[i].y = goals2.get(i).getY();
			pgs2[i].theta = goals2.get(i).getTheta();
		}
		DoubleByReference delayTe1 = new DoubleByReference();
		DoubleByReference delayTe2 = new DoubleByReference();

		INSTANCE.computeTimeDelay(p, ps1, pgs1, pgs1.length, ps2, pgs2, pgs2.length, robotID1, robotID2, delayTe1,delayTe2);
		//metaCSPLogger.info("Delay if x drives first: " + delayTe1.getValue());
		//metaCSPLogger.info("Delay if y drives first: " + delayTe2.getValue());
		return new Pair<Double, Double>(delayTe1.getValue(), delayTe2.getValue());
	}

	public Pair<Double,Double> computeTimeDelay(Pose start1, Pose goal1, Pose start2, Pose goal2, int robotID1, int robotID2 ){
		

		ArrayList<Pose> goals1 = new ArrayList<Pose>();
		goals1.add(goal1);

		ArrayList<Pose> goals2 = new ArrayList<Pose>();
		goals2.add(goal2);
		
		return computeTimeDelay(start1,goals1,start2,goals2, robotID1, robotID2);

	}
    
	public Pair<PoseSteering[], double[]> calculatePath(int robotID, Pose start, ArrayList<Pose> goals, double distanceBetweenPathPoints ){

		ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();  
		ArrayList<Double> finalCurvatures = new ArrayList<Double>();  
		for (int i = 0; i < goals.size(); i++) {
			Pose goal = goals.get(i);
			if (i > 0) start = goals.get(i-1);
			Pair<PoseSteering[], double[]> p =  calculatePath(robotID, start, goal, distanceBetweenPathPoints);
			PoseSteering[] pathPoses = p.getFirst();
			double[] curvatures = p.getSecond();

			if (i == 0) {
				finalPath.add(pathPoses[0]);
				finalCurvatures.add(curvatures[0]);
			}
			for (int j = 1; j < pathPoses.length; j++) {
				finalPath.add(pathPoses[j]);
				finalCurvatures.add(curvatures[j]);
			}
		}
		metaCSPLogger.info("Path Length: " + finalPath.size());
		PoseSteering[] path = finalPath.toArray(new PoseSteering[finalPath.size()]);
		double[] curvatures = ArrayUtils.toPrimitive(finalCurvatures.toArray(new Double[finalCurvatures.size()]));

		return new Pair<PoseSteering[], double[]>(path, curvatures);
	}
    
	public  Pair<PoseSteering[], double[]> calculatePath(int robotID, Pose start, Pose goal, double distanceBetweenPathPoints ){
		ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();  
		PointerByReference pathi = new PointerByReference();
		PointerByReference curvsi = new PointerByReference();
		IntByReference path_length = new IntByReference();
		PathPose ps = new PathPose();
		ps.x = start.getX();
		ps.y = start.getY();
		ps.theta = start.getTheta();
		PathPose pg = new PathPose();
		pg.x = goal.getX();
		pg.y = goal.getY();
		pg.theta = goal.getTheta();
		INSTANCE.calculatePath(p, robotID, ps, pg, pathi,curvsi, path_length,distanceBetweenPathPoints);
		metaCSPLogger.info("Path Length: " + path_length.getValue());	
		//Get the Path points
		final Pointer pathVals = pathi.getValue();
		final PathPose valsRef = new PathPose(pathVals);
		valsRef.read();
		int numVals = path_length.getValue();
		if (numVals == 0) return null;
		PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
		for (int j = 0; j < pathPoses.length; j++) {
			finalPath.add(new PoseSteering(pathPoses[j].x, pathPoses[j].y, pathPoses[j].theta, 0.0));
		}
		//Get the Path curvatures
		final Pointer pathCurvs = curvsi.getValue();
		double[] curvatures = pathCurvs.getDoubleArray(0, numVals);
		INSTANCE.cleanupPath(pathVals, pathCurvs);
		
		return new Pair<PoseSteering[], double[]>(finalPath.toArray(new PoseSteering[finalPath.size()]), curvatures);
	}
	
	/**
	 * Add a spatial envelope to the fleetmaster gridmap while giving its bounding box.
	 * @param robotID ID of the robot for which adding the new path.
	 * @param pathID ID of the path to be added.
	 * @param pathToAdd The path to be added.
	 * @param coordinates The polygon of the robot footprint (clockwise or anti clockwise coordinates).
	 * @return <code>true</code> if success.
	 */
	public boolean addPath(int robotID, int pathID, PoseSteering[] pathToAdd, Geometry boundingBox, Coordinate ... coordinates) {	
		if (p == null) {
			metaCSPLogger.severe("The fleetmaster has noot been initialized yet. Call init function first!");
			return false;
		}
		
		
		if (pathToAdd.length == 0) {
				metaCSPLogger.severe("Added paths cannot have have null length!");
				return false;
		}
		if (coordinates.length == 0) coordinates = DEFAULT_FOOTPRINT;	
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coordinates.length+1];
		for (int i = 0; i < coordinates.length; i++) {
			newCoords[i] = coordinates[i];
		}
		newCoords[newCoords.length-1] = coordinates[0];
		gf.createPolygon(newCoords);
		if (gf.createPolygon(newCoords).getArea() == 0) {
			metaCSPLogger.severe("Robots' footprint cannot have null area!");
			throw new Error("Robots' footprint cannot have null area!");
		}
			
		//already added
		if (paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			metaCSPLogger.warning("Path already stored.");
			return true;
		}
		
		//Add the new path
		clearPath(pathID);
		
		//Parse the Java values for the path and the footprint
		double[] coordinates_x = new double[coordinates.length];
		double[] coordinates_y = new double[coordinates.length];
		for (int i = 0; i < coordinates.length; i++) {
			coordinates_x[i] = coordinates[i].x;
			coordinates_y[i] = coordinates[i].y;
		}
		
		PathPose[] path = (PathPose[])new PathPose().toArray(pathToAdd.length);
		double[] steering = new double[pathToAdd.length];
		for (int i = 0; i < pathToAdd.length; i++) {
			path[i].x = pathToAdd[i].getX();
			path[i].y = pathToAdd[i].getY();
			path[i].theta = pathToAdd[i].getTheta();
			steering[i] = pathToAdd[i].getSteering();
		}
		
		
		Coordinate[] bbx = null;
		if (boundingBox != null) bbx = boundingBox.getCoordinates();
		else if (gridParams.dynamic_size) {
			SpatialEnvelope se = TrajectoryEnvelope.createSpatialEnvelope(pathToAdd, coordinates);
			boundingBox = se.getPolygon();
		}
				
		//Call the method. -1 is used as special value to indicate that the path was not correctly added.
		//Long pathCode = new Long(0 ); //FIXME Test?
		Long pathCode = Long.valueOf("0"); 
		TrajParams trjp = trajParams.containsKey(robotID) ? trajParams.get(robotID) : DEFAULT_TRAJ_PARAMS;
		double bottom_left_x = (bbx == null) ? Double.MAX_VALUE : bbx[0].x;
		double bottom_left_y = (bbx == null) ? Double.MAX_VALUE : bbx[0].y;
		double top_right_x = (bbx == null) ? Double.MAX_VALUE : bbx[2].x;
		double top_right_y = (bbx == null) ? Double.MAX_VALUE : bbx[2].y;
		INSTANCE.addPathInterface(p, path);
		//pathCode = Long.valueOf(INSTANCE.addPathInterface(path);
		paths.put(pathID, pathCode);

		metaCSPLogger.info("Adding path RobotID: " + robotID + ", pathID: " + pathID + ", fleetmaster pathID: " + pathCode + ".");
		
		//return !pathCode.equals(new Long(0));
		return !pathCode.equals(Long.valueOf("0"));
	}
	
	/**
	 * Add a trajectory envelope to the fleetmaster gridmap while giving its bounding box.
	 * @param te The trajectory envelope to add.
	 * @return <code>true</code> if success.
	 */
	public boolean addPath(TrajectoryEnvelope te) {
		if (te.getEnvelopeBoundingBox() == null) {
			metaCSPLogger.severe("The given TrajectoryEnvelope is empty!");
			throw new Error("The given TrajectoryEnvelope is empty!");
		}
		if (te.getEnvelopeBoundingBox().getArea() <= 0) {
			metaCSPLogger.severe("Robots' footprint cannot have null area!");
			throw new Error("Robots' footprint cannot have null area!");
		}
		return addPath(te.getRobotID(), te.getID(), te.getTrajectory().getPoseSteering(), te.getEnvelopeBoundingBox(), te.getFootprint().getCoordinates());
	}

	
	/**
	 * Remove the path from the fleetmaster GridMaps.
	 * @param pathID The ID of the path to be cleared.
	 */
	public boolean clearPath(int pathID) {
		if (p != null && paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			INSTANCE.removePath(p, paths.get(pathID));
			paths.remove(pathID);
			metaCSPLogger.info("Clearing path pathID: " + pathID);
			return true;
		}
		return false;
	}
	
	/**
	 * Update the current progress along the path (according to the last received robot report).
	 * @param pathID The path ID.
	 * @param currentIdx The current path index.
	 * @return <code>true</code> if the path index has been correctly updated.
	 */
	public boolean updateCurrentPathIdx(int pathID, int currentIdx) {
		if (p != null && paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			metaCSPLogger.info("Updating path pathID: " + pathID + ", current path index: " + currentIdx + ".");
			return INSTANCE.updateCurrentPathIdx(p, paths.get(pathID), new Long(Math.max(currentIdx, 0)));
		 }
		 return false;
	}
	

	

}
