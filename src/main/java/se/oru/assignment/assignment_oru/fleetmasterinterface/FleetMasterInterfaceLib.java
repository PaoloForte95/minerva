package se.oru.assignment.assignment_oru.fleetmasterinterface;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.sun.jna.Library;
import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;




public interface FleetMasterInterfaceLib extends Library {
			
	
	
	PointerByReference init(EnvelopeGridParams gridParams);
	
	void setTrajectoryParams(PointerByReference p, int robotID, TrajParams trajParams, String debugPrefix );

	void setVehicleType(PointerByReference p, int robotID, VehicleModel2d vehicleType);
	
	void addPathInterface(PointerByReference p, PathPose[] path);
	
    void removePath(PointerByReference p, long id);
    
    long sizeDeltaTVec();

    boolean updateCurrentPathIdx(PointerByReference p, long pathId, long currentIdx);
    
    void queryTimeDelay(PointerByReference p, long pathId1, long pathId2, int csStart1, int csEnd1, int csStart2, int csEnd2,
    				long[] indicesTCD1, double[] valuesTCD1, long sizeTCD1, long[] indicesTCD2, double[] valuesTCD2, long sizeTCD2, DoubleByReference delay1, DoubleByReference delay2);
    

	void computeTimeDelayWPath(PointerByReference p, PathPose[] path1, int path1_length, double[] curvs1, PathPose[] path2, int path2_length, double[] curvs2, int robotID1, int robotID2, DoubleByReference delay1, DoubleByReference delay2 );
    
	void computeTimeDelay(PointerByReference p, PathPose start1, PathPose[] goals1, int numgoals1, PathPose start2, PathPose[] goals2, int numgoals2, int robotID1, int robotID2, DoubleByReference delay1, DoubleByReference delay2 );
	
	void calculatePath(PointerByReference p, int robotID, PathPose start, PathPose goal, PointerByReference path, PointerByReference curvatures, IntByReference pathLength, double distanceBetweenPathPoints);

	public void cleanupPath(Pointer p);
    

	
	public  class PathPose extends Structure {
		
		public double x;
		public double y;
		public double theta;
		
		public PathPose() {}
		public PathPose(double x, double y, double theta) {
			this.x = x; 
			this.y = y;
			this.theta = theta;
		}
		public PathPose(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"x", "y", "theta"});
		}
	}

	public static class VehicleModel2d extends Structure {
		public static class ByReference extends VehicleModel2d implements Structure.ByReference {}
		
		public static enum VehicleModel2dType {Articulated, CarLike, Omni4sd} ;
		public int type;
		
		public VehicleModel2d() {}
		public VehicleModel2d(Pointer p) {
			super(p);
		}
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"type"});
		}
		
	}
	

	public class EnvelopeGridParams extends Structure  {
		
		public Pointer derivedClass1;  //Necessary to map correctly the field
		public Pointer derivedClass2;  //Necessary to map correctly the field
		public PathPose origin = new PathPose(0,0,0);
		public double resolution = 0.1;
		public NativeLong width = new NativeLong(100);
		public NativeLong height = new NativeLong(100);
		public boolean dynamic_size = false;
		public int robotID = -1;
		public boolean debug = false;
		
		
		public EnvelopeGridParams() {}
		public EnvelopeGridParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			List<String> fields = new ArrayList<String>();
			for (Field field : EnvelopeGridParams.class.getFields()){
				String fieldName =  field.getName();
				if(!fieldName.contains("ALIGN_DEFAULT") && !fieldName.contains("ALIGN_GNUC") && !fieldName.contains("ALIGN_MSVC") && !fieldName.contains("ALIGN_NONE") ){
					fields.add(fieldName);
				}
			}
			return fields;
		}
	}
	
	public static class TrajParams extends Structure {
		public double derivedClass1; //Necessary to map correctly the field
		public int type = 1;
		public double maxVel = 1.0;
		public double maxVelRev = maxVel;
		public boolean useSteerDriveVel = true;
	    public double maxRotationalVel = 1.;
	    public double maxRotationalVelRev = maxRotationalVel;
	    public double maxSteeringAngleVel = 1.;
		public double initVel = 0.;
		public double endVel = 0.;
		public double initSteeringAngleVel = 0.;
		public double endSteeringAngleVel = 0.;
	    public double maxAcc = 1.;
	    public double maxRotationalAcc = 1.;
	    public double maxSteeringAngleAcc = 1.;
		public double timeStep = 0.06;
		public double wheelBaseX = 0.68; 
		public double wheelBaseY = 0.;
		public boolean useInitialState = true;
		public double minDist = 0.00001;
		public boolean useCoordTimeAccConstraints = true;
		public int nbZeroVelControlCommands = 5;
		public boolean useCoordTimeContraintPoints = false;
		public double creepSpeed = 0.;
		public double creepDistance = 0.;
		public boolean setCreepSpeedAsEndConstraint = false;
		public int citiTruckNbClearSpeedCommands = 0;
		public boolean debug = false;
		public String debugPrefix = "";

		public TrajParams() {}
		public TrajParams(Pointer p) {
			super(p);
			
		}
		@Override
		protected List<String> getFieldOrder() {
			List<String> fields = new ArrayList<String>();
			for (Field field : TrajParams.class.getFields()){
				String fieldName =  field.getName();
				if(!fieldName.contains("ALIGN_DEFAULT") && !fieldName.contains("ALIGN_GNUC") && !fieldName.contains("ALIGN_MSVC") && !fieldName.contains("ALIGN_NONE") ){
					fields.add(fieldName);
				}
			}
			return fields;
			
		}
		
	}
	
}
