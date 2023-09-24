package se.oru.assignment.assignment_oru.methods;

import java.util.ArrayList;
import java.util.Calendar;

import org.metacsp.utility.logging.MetaCSPLogging;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.problems.AbstractOptimization;
import se.oru.assignment.assignment_oru.problems.AbstractOptimization.problemStatus;

/**
 * This class provide a systematic algorithm to solve an optimization problem defined with the class {@link se.oru.assignment.assignment_oru.problems.AbstractOptimizationProblem#buildOptimizationProblem}.
 * 
 * @author pofe
 *
 */

public final class SystematicAlgorithm extends AbstractOptimizationAlgorithm {


	public SystematicAlgorithm(){
		super();
		logger = MetaCSPLogging.getLogger(this.getClass());
	}

	protected Pair <Double, Double> computeAssignmentCost(AbstractOptimization oap){
		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double costofAssignment = 0;
		double costofAssignmentForConstraint = 0;
		double costF = 0;
		double [][][] costValuesMatrix = oap.getInterferenceFreeCostMatrix();
		double linearWeight = oap.getLinearWeight();
		//Take time to understand how much time require this function
		for (int robotID : IDsAllRobots ) {
			int i = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsAllTasks ) {
				int j = IDsAllTasks.indexOf(taskID);
				for(int s = 0; s < maxNumPaths; s++) {
					if ( oap.getAssignmentMatrix()[i][j][s] > 0) {

						if (linearWeight != 1) {
							//Evaluate cost of F function only if alpha is not equal to 1
							//double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
							double costB = oap.getRobotSingleCost(robotID,taskID,s);
							//fileStream11.println(" ");
							costF = oap.evaluateInterferenceCost(robotID,taskID,s);
							costofAssignment = linearWeight*costB + (1-linearWeight)*costF + costofAssignment ;
							costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costF + costofAssignmentForConstraint;
						}
						else {
							//double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
							double costB = oap.getRobotSingleCost(robotID,taskID,s);
							costofAssignment = Math.pow(linearWeight*costB, 2) + costofAssignment ;
							costofAssignmentForConstraint = costValuesMatrix[i][j][s]  + costofAssignmentForConstraint;
						}


					}
				}

			}		
		}		
	return new Pair<Double, Double>(costofAssignment,costofAssignmentForConstraint);
	}

	protected int [][][] copyData(int[][][] AssignmentMatrix){

		int [][][] optimalAssignmentMatrix = new int[AssignmentMatrix.length][AssignmentMatrix[0].length][AssignmentMatrix[0][0].length];
		for(int i=0; i< AssignmentMatrix.length;i ++) {
			for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
				for(int s = 0; s < AssignmentMatrix[0][0].length; s++) {
					optimalAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];

				}
			}
		}
		return optimalAssignmentMatrix;
	}

	/** 
	 * Solve the optimization problem given as input using a systematic algorithm. The objective function is defined as alpha*B + (1-alpha)*F.
	 * The solver first finds the optimal solution considering only B function and then
	 * for each solution (i.e. a sub-optimal assignment) evaluates the cost of F function. Each new sub-optimal solution (i.e. with the cost less that then previous sub-optimal one)
	 * introduces a new constraint on cost for the next solutions. This constraint prunes all solutions that have a cost considering only B higher that the sub-optimal(that consider B + F) 
	 * since none of these solutions can be the optimal one.
	 * @param oap -> an optimization problem defined with {@link se.oru.assignment.assignment_oru.problems.AbstractOptimizationProblem#buildOptimizationProblem}

	 * @return An Optimal Assignment that minimize the objective function
	 */	
	public int [][][] solveOptimizationProblem(AbstractOptimization oap){
		logger.info("Solving the problem using the Systematic Algorithm");
		long initialTime = Calendar.getInstance().getTimeInMillis();
		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();

		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();

		//Initialize the optimal assignment and the cost associated to it
		int [][][] optimalAssignmentMatrix = new int[numRobotAug][numTaskAug][maxNumPaths];
		double currentOptimalCost = objectiveOptimalValue;
		//Solve the optimization problem
		problemStatus resultStatus = problemStatus.NOT_SOLVED;

		while(resultStatus != problemStatus.INFEASIBLE && computationalTime < getMaxComputationalTime()) {
			//Evaluate an optimal assignment that minimize only the B function
			resultStatus = oap.solve();
			if(resultStatus == problemStatus.INFEASIBLE){
				break;
			}
			solutionsEvaluated +=1 ;
			//Evaluate the overall costs of the assignment
			int [][][] AssignmentMatrix = oap.getAssignmentMatrix();
			Pair<Double, Double> costs = computeAssignmentCost(oap);
			double costofAssignment = costs.getFirst();
			double costofAssignmentForConstraint = costs.getSecond();
			//Compare actual solution and optimal solution finds so far
			if (costofAssignment < currentOptimalCost && resultStatus != problemStatus.INFEASIBLE) {
				currentOptimalCost = costofAssignment;
				optimalAssignmentMatrix = copyData(AssignmentMatrix);
			}
			//Add the constraint on cost for next solution
			//add +0,005 in order for tolerance
			oap.constraintOnCostSolution(costofAssignmentForConstraint);
			//Add the constraint to actual solution in order to consider this solution as already found  
			computationalTime =  Calendar.getInstance().getTimeInMillis() - initialTime;
		}

		long timeFinal = Calendar.getInstance().getTimeInMillis();
		computationalTime = (timeFinal- initialTime)/1000;
		logger.info("Number of solution evaluated: " + solutionsEvaluated + " in " + computationalTime + " seconds.");
		//Return the Optimal Assignment Matrix 
		return  optimalAssignmentMatrix;    
	}


}

