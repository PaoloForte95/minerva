package se.oru.assignment.assignment_oru.methods;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.problems.AbstractDelayEvaluator;

public class SimpleDelayEvaluator extends AbstractDelayEvaluator {

    public SimpleDelayEvaluator(){
        super();
    }

 @Override
    public Pair<Double, Double> evaluatePathDelay(int robot1ID, PoseSteering[] pss1, int robot2ID, PoseSteering[] pss2) {
        return new Pair<Double, Double> (1.0, 1.0);
    }
    
}
