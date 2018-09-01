package solver;

import com.sun.istack.internal.NotNull;
import org.omg.PortableServer.LIFESPAN_POLICY_ID;
import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Random;

class ChangeLog {
    boolean boxType;
    boolean indexType;

    int boxIndex;
    double newpos;

    ChangeLog(List<Box> movingBoxes, List<Box> movingObs) {
        Random r = new Random(Calendar.getInstance().getTimeInMillis());
        boxType = r.nextFloat() < 0.5;
        indexType = r.nextFloat() < 0.5;

        // random chose from boxes and obstacles
        if(boxType) {
            boxIndex = r.nextInt(movingBoxes.size());// move boxes
        } else {
            boxIndex = r.nextInt(movingObs.size());// move boxes
        }
        newpos = r.nextDouble(); // double [0, 1]
    }

    ChangeLog(boolean boxType, boolean indexType, int boxIndex, double newpos) {
        this.boxIndex = boxIndex;
        this.boxType = boxType;
        this.indexType = indexType;
        this.newpos = newpos;
    }

}

public class RoadmapForBox implements Roadmap {
    private ProblemSpec ps;
    List<Vertex> map;
    //private int boxIndex;
    static final int BOX_MOVE = 0;
    static final int OBS_MOVE = 1;

    ChangeLog cLog;

    public RoadmapForBox(ProblemSpec ps) {
        this.ps = ps;
        map = new ArrayList<>();
        map.add(new Vertex(ps.getMovingBoxes(), ps.getMovingObstacles()));
        cLog = null;
    }

    /**
     * Add the new vertex to the Road map
     * First find the closest Vertex(n) to Vertex(v) in the current map, and try to
     * add a new vertex(q) in line segment (n, v) ensuring:
     * 1. length(n, q) <= D_MAX
     * 2. line segment(n, q) is collision free
     *
     * Then add the Vertex to the map as a child node of n
     */
    @Override
    public boolean addNewVertex(Vertex v) throws InsertFailedException {
        if(v == null) {
            throw new InsertFailedException();
        }
        Vertex near;
        Vertex newNode;
        // naive method, time: O(n)
        near = findNearest(v);
        // naive method, try randomly and check
        if(near != null) {
            newNode = findNextConfigOn(near, v);
        }
        else {
            throw new InsertFailedException();
        }

        if(newNode == null)
            throw new InsertFailedException();

        // append new node as a child node of a predecessor
        appendVertex(near, newNode);
        return false;
    }


    private Vertex findNearest(Vertex v) {
        Vertex res = null;
        double minDis = Double.MAX_VALUE;
        for(Vertex vertex : map) {
            if(distance(vertex, v) < minDis) {
                minDis = distance(vertex, v);
                res = vertex;
            }
        }
        return res;
    }

    private void appendVertex(@NotNull Vertex predecessor, Vertex children) {
        children.setParent(predecessor);
        map.add(children);
    }

    private double distance(Vertex a, Vertex b) {
        return 0;
    }

    //find a new Configuration on line segment (a, b)
    private Vertex findNextConfigOn(Vertex a, Vertex b) {
        return null;
    }

    @Override
    public Vertex newVertex() {
        Vertex v;
        int MAX_ITER = 1000;
        do {
            v = sample();
        } while(--MAX_ITER != 0 && collisionCheck(v));

        if(MAX_ITER == 0) return null; // max_iteration exceed
        else return v;
    }

    @Override
    public void printTrace() {
        System.out.println("not implemented");
    }

    /**
     * sample() use random sampling to sample a configuration
     * and set the ChangeBlog to a new one
     * @return
     */
    private Vertex sample() {
        List<Box> movingBox = new ArrayList<>(ps.getMovingBoxes());
        List<Box> movingObs = new ArrayList<>(ps.getMovingObstacles());

        cLog = new ChangeLog(movingBox, movingObs);

        // random chose from boxes and obstacles
        List<Box> changed;
        if(cLog.boxType) {
            changed = movingBox;
        } else {
            changed = movingObs;
        }

        Box boxToMove = changed.get(cLog.boxIndex);
        if(cLog.indexType) {
            boxToMove.pos.setLocation(cLog.newpos, boxToMove.pos.getY()); // change X
        } else {
            boxToMove.pos.setLocation(boxToMove.pos.getX(), cLog.newpos); // change Y
        }

        return new Vertex(movingBox, movingObs);
    }



    /** Collision check for a state */
    private boolean collisionCheck(Vertex v) {
        List<Box> movingBoxes = v.getMovingBoxes();
        List<Box> movingObs = v.getMovingObstacles();
        List<StaticObstacle> staticObs = ps.getStaticObstacles();

        for(int i = 0; i < movingObs.size(); i++) {
            for(int j = 0; j < movingObs.size(); j++) {
                if(i == j) continue;
                if (movingObs.get(i).getRect()
                        .intersects(movingObs.get(j).getRect())) {
                    return true;
                }
            }

            for(int j = 0; j < movingBoxes.size(); j++) {
                if(movingObs.get(i).getRect()
                        .intersects(movingBoxes.get(j).getRect())) {
                    return true;
                }
            }

            for(int j = 0; j < staticObs.size(); j++) {
                if(movingObs.get(i).getRect()
                        .intersects(staticObs.get(j).getRect())) {
                    return true;
                }
            }
        }

        for(int i = 0; i < movingBoxes.size(); i++) {
            for(int j = 0; j < movingObs.size(); j++) {
                if (movingBoxes.get(i).getRect()
                        .intersects(movingObs.get(j).getRect())) {
                    return true;
                }
            }

            for(int j = 0; j < movingBoxes.size(); j++) {
                if(i == j) continue;
                if(movingBoxes.get(i).getRect()
                        .intersects(movingBoxes.get(j).getRect())) {
                    return true;
                }
            }

            for(int j = 0; j < staticObs.size(); j++) {
                if(movingBoxes.get(i).getRect()
                        .intersects(staticObs.get(j).getRect())) {
                    return true;
                }
            }
        }
        return false;
    }

    @Deprecated
    private boolean collisionCheck(Vertex curr, Vertex next) {
        return false;
    }


    /*
    private boolean collisionCheck(Vertex curr) {
        if(cLog == null) return true;

        return false;
    }
    */

}
