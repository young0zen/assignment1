package solver;

import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;

import java.util.ArrayList;
import java.util.List;

public class Vertex {

    private Vertex parent;
    private ChangeLog cLog;

    //current configuration
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;

    public Vertex getParent() {
        return parent;
    }

    public ChangeLog getLog() {
        return cLog;
    }

    public void setParent(Vertex parent) {
        this.parent = parent;
    }

    Vertex(List<Box> movingBoxes, List<Box> movingObstacles) {
        this.movingBoxes = new ArrayList<>(movingBoxes);
        this.movingObstacles = new ArrayList<>(movingObstacles);
        parent = null;
        cLog = null;
    }

    Vertex(List<Box> movingBoxes) {
        this.movingBoxes = new ArrayList<>(movingBoxes);
        this.movingObstacles = null;
        parent = null;
        cLog = null;
    }

    Vertex(List<Box> movingBoxes, List<Box> movingObstacles, ChangeLog log) {
        this.movingBoxes = new ArrayList<>(movingBoxes);
        this.movingObstacles = new ArrayList<>(movingObstacles);
        parent = null;
        cLog = log;
    }

    public Vertex newVertexSingleMove(ChangeLog log, Box newBox) {
        List<Box> movingBox = new ArrayList<>(movingBoxes);
        List<Box> movingObs = new ArrayList<>(movingObstacles);

        if(log.boxType) {
            movingBox.set(log.boxIndex, newBox);
        } else {
            movingObs.set(log.boxIndex, newBox);
        }
        Vertex v = new Vertex(movingBox, movingObs, log);
        v.setParent(this);
        return v;
    }

    public Vertex newVertexSingleMove(int index, MovingBox newBox) {
        List<Box> movingBox = new ArrayList<>(movingBoxes);
        Box oldBox = movingBox.get(index);
        movingBox.set(index, newBox);
        ChangeLog log = new ChangeLog(index, (MovingBox)oldBox, newBox);
        Vertex v = new Vertex(movingBox, movingObstacles, log);
        v.setParent(this);
        return v;
    }

    public Vertex newVertexSingleMove(int index, MovingObstacle newBox) {
        List<Box> movingObs = new ArrayList<>(movingObstacles);
        Box oldBox = movingObs.get(index);
        movingObs.set(index, newBox);
        ChangeLog log = new ChangeLog(index, (MovingObstacle)oldBox, newBox);
        Vertex v = new Vertex(movingBoxes, movingObs, log);
        v.setParent(this);
        return v;
    }

    public double heuristic(Vertex goal) {
        double h = 0.0;

        for(Box boxCurr : movingBoxes) {
            double minv = Double.MAX_VALUE;
            for(Box boxGoal : goal.getMovingBoxes()) {
                minv = Math.min(minv, boxCurr.getPos().distance(boxGoal.getPos()));
            }
            h += minv;
        }
        return h;
    }

    public List<Box> getMovingBoxes() {
        return movingBoxes;
    }

    public List<Box> getMovingObstacles() {
        return movingObstacles;
    }

    public double distanceTo(Vertex v) {
        return -1;
    }

}
