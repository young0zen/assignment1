package solver;

import problem.Box;

import java.util.List;

public class Vertex {
    private Vertex parent;

    //current configuration
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;

    public Vertex getParent() {
        return parent;
    }

    public void setParent(Vertex parent) {
        this.parent = parent;
    }


    Vertex(List<Box> movingBoxes, List<Box> movingObstacles) {
        this.movingBoxes = movingBoxes;
        this.movingObstacles = movingObstacles;
        parent = null;
    }

    public List<Box> getMovingBoxes() {
        return movingBoxes;
    }

    public List<Box> getMovingObstacles() {
        return movingObstacles;
    }


}
