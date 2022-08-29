package se.oru.assignment.assignment_oru.test;


import java.util.Calendar;


import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import se.oru.coordination.coordination_oru.demo.DemoDescription;




@DemoDescription(desc = "Minimal example of multi robot task assignment with an empty map; 5 robots and 5 tasks")
public class TestOptimization {
	//load library used for optimization (or tools)
	protected static MPVariable [][][] tranformArray(MPSolver optimizationProblem,int numRobotAug, int numTaskAug,int alternativePaths) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobotAug][numTaskAug][alternativePaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for (int s = 0; s < alternativePaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTaskAug*alternativePaths+j*alternativePaths+s];
				 }
				 
			 }
	    }
		return decisionVariable;
	}
	
	public static MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][][] assignmentMatrix, int numRobotAug, int numTaskAug,int alternativePaths) {
		//Take decision Variable from Optimization Problem
		MPVariable [][][] DecisionVariable = tranformArray(optimizationProblem, numRobotAug, numTaskAug, alternativePaths);
		//Initialize a Constraint
		//MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		MPConstraint c2 = optimizationProblem.makeConstraint(0,numRobotAug-1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < alternativePaths; s++) {
    					if (assignmentMatrix[i][j][s] >0) {
    						c2.setCoefficient(DecisionVariable[i][j][s],1);
    					}
    			}
    		}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
	
	 static {
		    Loader.loadNativeLibraries();
	}
	public static void main(String[] args) throws InterruptedException {
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		int numRobotAug = 10;
		int numTaskAug = 10;
		int alternativePaths = 1;
		//MPSolver optimizationProblem = new MPSolver(
				//"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] decisionVariable = new MPVariable[numRobotAug][numTaskAug][alternativePaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 decisionVariable[i][j][s] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+","+s+"]");
					
				 }
				
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < numRobotAug; i++) {
			 //Initialize the constraint
			 //MPConstraint c0 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY, 1);
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1);
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 }
				
			 }
		 }
		
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTaskAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 } 		
			 }
		 }
		 MPObjective objective = optimizationProblem.objective();
		 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 double singleRobotCost  =  100;
					 if ( singleRobotCost != Double.POSITIVE_INFINITY) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.setCoefficient(decisionVariable[i][j][s], singleRobotCost); 
					 }else { // if the path does not exists or the robot type is different from the task type 
						//the path to reach the task not exists
						//the decision variable is set to 0 -> this allocation is not valid
						MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
						c3.setCoefficient(decisionVariable[i][j][s],1); 
					 }
				 }
			 }			 
		 }
		 long startTime = Calendar.getInstance().getTimeInMillis();
		 MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		 while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			 resultStatus = optimizationProblem.solve();
			 MPVariable [][][] decisionVariable2 = tranformArray(optimizationProblem, numRobotAug, numTaskAug, alternativePaths);
				double [][][] assignmentMatrix = new double [numRobotAug][numTaskAug][alternativePaths];	
				//Store decision variable values in a Matrix
				for (int i = 0; i < numRobotAug; i++) {
					for (int j = 0; j < numTaskAug; j++) {
						for(int s = 0;s < alternativePaths; s++) {
							assignmentMatrix[i][j][s] = decisionVariable2[i][j][s].solutionValue();
							//System.out.println("Value i:" + i + " Value j: " + j + " Value s: " +s + " is = " +  assignmentMatrix[i][j][s] );
						}
					}
				}
				optimizationProblem = constraintOnPreviousSolution(optimizationProblem,assignmentMatrix, numRobotAug, numTaskAug, alternativePaths);
		 }
		 long finalTime = Calendar.getInstance().getTimeInMillis();
		 long totalTime = finalTime - startTime;
		 System.out.println("Time to evaluate all the possible solutions: " + totalTime/1000 + " seconds");
	}
}
