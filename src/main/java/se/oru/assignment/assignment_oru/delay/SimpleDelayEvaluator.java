package se.oru.assignment.assignment_oru.delay;

import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;

import aima.core.util.datastructure.Pair;

public class SimpleDelayEvaluator implements AbstractDelayEvaluator {
    protected Logger logger = MetaCSPLogging.getLogger(this.getClass());
    public SimpleDelayEvaluator(){
        super();
    }

 @Override
    public Pair<Double, Double> evaluatePathDelay(int robot1ID, PoseSteering[] pss1, int robot2ID, PoseSteering[] pss2) {
        return new Pair<Double, Double> (1.0, 1.0);
    }

@Override
public Pair<Double, Double> evaluatePathDelay(int robot1id, PoseSteering[] robot1ID, double[] curvs1, int robot2ID, PoseSteering[] pss2, double[] curvs2) {
    return new Pair<Double, Double> (1.0, 1.0);
}
    
}
