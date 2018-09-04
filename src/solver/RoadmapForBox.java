package solver;

import com.sun.istack.internal.NotNull;
import problem.*;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

import static java.lang.Math.abs;

//TODO: there is a goal state for every single box you can't just rearrange them
//TODO: getPos() give the bottom-left corner, not center
class ChangeLog {
    boolean boxType; // true for movingBox, false for moving obstacles
    boolean indexType;

    int boxIndex;
    int nodeIndex = -1;
    double newpos;

    final Box oldBox; // parent.get(boxIndex)
    //Box newBox;

    ChangeLog(List<Vertex> map) {
        Random r = new Random(Calendar.getInstance().getTimeInMillis());
        boxType = r.nextFloat() < 0.5;
        indexType = r.nextFloat() < 0.5;
        nodeIndex = r.nextInt(map.size());

        Vertex parent = map.get(nodeIndex);
        List<Box> movingBoxes = parent.getMovingBoxes();
        List<Box> movingObs = parent.getMovingObstacles();

        double width;
        // random chose from boxes and obstacles
        if(boxType) {
            boxIndex = r.nextInt(movingBoxes.size());// move boxes
            oldBox = movingBoxes.get(boxIndex);
            width = oldBox.getWidth();
        } else {
            boxIndex = r.nextInt(movingObs.size());// move boxes
            oldBox = movingObs.get(boxIndex);
            width = oldBox.getWidth();
        }
        // double [0, 1 - width]
        newpos = r.nextDouble() * (1 - width);
    }

    ChangeLog(ChangeLog oldLog, double newpos) {
        this.indexType = oldLog.indexType;
        this.boxType = oldLog.boxType; // not going to chang it
        this.oldBox = oldLog.oldBox;
        this.nodeIndex = oldLog.nodeIndex;
        this.boxIndex = oldLog.boxIndex;
        this.newpos = newpos;
    }

    ChangeLog(int index, MovingBox ob, MovingBox nb) {
        this.boxIndex = index;
        this.boxType = true;
        this.oldBox = ob;
        if(abs(nb.getPos().getY() - ob.getPos().getY()) > 0.001) {
            this.newpos = nb.getPos().getY();
            this.indexType = false;
        } else {
            this.newpos = nb.getPos().getX();
            this.indexType = true;
        }
    }

    ChangeLog(int index, MovingObstacle ob, MovingObstacle nb) {
        this.boxIndex = index;
        this.boxType = false;
        this.oldBox = ob;
        if(abs(nb.getPos().getY() - ob.getPos().getY()) > 0.001) {
            this.newpos = nb.getPos().getY();
            this.indexType = false;
        } else {
            this.newpos = nb.getPos().getX();
            this.indexType = true;
        }
    }
}

public class RoadmapForBox implements Roadmap {

    private ProblemSpec ps;
    Vertex goal;
    List<Vertex> map;
    //private int boxIndex;
    static final double PRIMITIVE = 0.01; // distance of primitive move
    static final double MAX_EDGE_LENGTH = 0.5;
    static final double CLOSE_HEURISTIC = 0.5;
    static final int MAX_ITER = 10000;

    ChangeLog cLog;

    public RoadmapForBox(ProblemSpec ps) {
        this.ps = ps;
        initGoal();
        map = new ArrayList<>();
        map.add(new Vertex(ps.getMovingBoxes(), ps.getMovingObstacles()));
        cLog = null;
    }

    private void initGoal() {
        List<Point2D> points = ps.getMovingBoxEndPositions();
        List<Box> boxes = new ArrayList<>();
        for(Point2D point : points) {
            boxes.add(new MovingBox(point, ps.getRobotWidth()));
        }
        goal = new Vertex(boxes);
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
        if(near == null) {
            throw new InsertFailedException();
        }

        newNode = findNextConfigOn(near, v);

        // append new node as a child node of a predecessor
        if(newNode != null) {
            appendVertex(near, newNode);
            if(newNode.heuristic(goal) < CLOSE_HEURISTIC) {
                return tryConnectGoal(newNode);
            }
        }

        return false;
    }


    /** Find a nearest one in the map, time O(n) */
    /*
    private Vertex findNearest(Vertex v) {
        Vertex res = null;
        double minDis = Double.MAX_VALUE;
        for(Vertex vertex : map) {
            if(vertex.distanceTo(v) < minDis) {
                minDis = vertex.distanceTo(v);
                res = vertex;
            }
        }
        return res;
    }
    */

    /** Use node in the changelog as the nearest */
    private Vertex findNearest(Vertex v) {
        if(v.getLog() == null) return null;
        else return v.getParent();
    }

    private void appendVertex(@NotNull Vertex predecessor, Vertex children) {
        children.setParent(predecessor);
        map.add(children);
    }


    /** Find a new Configuration on line segment (a, b),
     * where a is the start position(i.e. try to reach from a), and b
     * is the end position*/
    private Vertex findNextConfigOn(Vertex a, Vertex b) {
        if(a == b.getParent()) {
            ChangeLog log = b.getLog();
            double distance;
            if(log.indexType) {
                distance = log.newpos - log.oldBox.getPos().getX();
            } else {
                distance = log.newpos - log.oldBox.getPos().getY();
            }
            distance = abs(distance);

            Box oldBox = log.oldBox;
            Box newBox;
            if(log.boxType) {
                newBox = b.getMovingBoxes().get(log.boxIndex);
            } else {
                newBox = b.getMovingObstacles().get(log.boxIndex);
            }

            while(distance > PRIMITIVE) { // TODO: condition
                if(!collisionExist(log, a, newBox) && distance < MAX_EDGE_LENGTH) {
                    double newpos = log.indexType ? newBox.getPos().getX() : newBox.getPos().getY();
                    log = new ChangeLog(log, newpos);
                    return a.newVertexSingleMove(log, newBox);
                } else {
                    if(log.boxType) {
                        newBox = new MovingBox(new Point2D.Double(
                                (oldBox.getPos().getX() + newBox.getPos().getX()) / 2,
                                (oldBox.getPos().getY() + newBox.getPos().getY()) / 2
                        ), newBox.getWidth()) {};
                    } else {
                        newBox = new MovingObstacle(new Point2D.Double(
                                (oldBox.getPos().getX() + newBox.getPos().getX()) / 2,
                                (oldBox.getPos().getY() + newBox.getPos().getY()) / 2
                        ), newBox.getWidth()) {};
                    }

                    distance /= 2;
                }
            }
            return null; // nothing found
        } else {
            return null; // TODO: not implemented
        }
    }

    private boolean tryConnectGoal(Vertex v) {
        List<Box> movingBoxes = v.getMovingBoxes();
        List<Box> goalState = new LinkedList<>(goal.getMovingBoxes());
        LinkedList<Vertex> feasible = new LinkedList<>();
        feasible.add(v);

        for(Box currBox : movingBoxes) {
            // the last Vertex in the list included the last successful box movement
            // this method only keep track of boxes, so while some of the boxes
            // in currV has been changed, the rest remains where they are.
            Vertex currV = feasible.getLast();
            //Greedy method, choose the one that has minimum distance
            Box target = null;
            double minv = Double.MAX_VALUE;
            int index = -1;
            for(Box goalBox : goalState) {
                if(currBox.getPos().distance(goalBox.getPos()) < minv) {
                    minv = currBox.getPos().distance(goalBox.getPos());
                    target = goalBox;
                }
                index++;
            }

            MovingBox inter = new MovingBox(new Point2D.Double(
                    currBox.getPos().getX(),
                    target.getPos().getY()
            ), currBox.getWidth());
            Vertex interV = currV.newVertexSingleMove(index, inter);

            boolean flag1 = true, flag2 = true;
            if(collisionExist(index, currV, inter) ||
                    collisionExist(index, interV, (MovingBox) target)) {
                flag1 = false; // no way from this side
            }

            inter = new MovingBox(new Point2D.Double(
                    currBox.getPos().getY(),
                    target.getPos().getX()
            ), currBox.getWidth());
            interV = currV.newVertexSingleMove(index, inter);

            if(collisionExist(index, currV, inter) ||
                    collisionExist(index, interV, (MovingBox) target)) {
                flag2 = false; // no way through here
            }

            if(flag1 == false && flag2 == false) {
                return false; // no way any where
            }

            feasible.add(interV); // okay store it
            feasible.add(interV.newVertexSingleMove(index, (MovingBox) target));
            goalState.remove(target);
        }
        //add all feasible solution to map
        for(Vertex vertex : feasible) {
            map.add(vertex);
        }
        return true;
    }


    @Override
    public Vertex newVertex() {
        Vertex v;
        int count = 0;
        do {
            v = sample();
        } while(collisionExist(v) && ++count != MAX_ITER);

        if(MAX_ITER == count) return null; // max_iteration exceed
        else return v;
    }

    @Override
    public void printTrace() {
        //TODO:
        System.out.println("not implemented");
    }

    /**
     * sample() use random sampling to sample a configuration
     * and set the ChangeBlog to a new one
     * @return the sampled vertex, not ensuring it's collision free
     */
    private Vertex sample() {
        ChangeLog cLog = new ChangeLog(map);

        Point2D newPoint;
        if(cLog.indexType) {
            newPoint = new Point2D.Double(cLog.newpos, cLog.oldBox.pos.getY());
        } else {
            newPoint = new Point2D.Double(cLog.oldBox.pos.getX(), cLog.newpos);
        }

        // random chose from boxes and obstacles
        Box newBox;
        if(cLog.boxType) {
            newBox = new MovingBox(newPoint, cLog.oldBox.getWidth());
        } else {
            newBox = new MovingObstacle(newPoint, cLog.oldBox.getWidth());
        }
        return map.get(cLog.nodeIndex).newVertexSingleMove(cLog, newBox);
    }



    /** Collision check for a state */
    private boolean collisionExist(Vertex v) {
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


    /** This method is only valid when curr is collision free!! */
    private boolean collisionExist(ChangeLog log, Vertex curr, Box newBox) {
        if(log == null) {
            return true;
        }
        if(log.boxType) {
            return collisionExist(log.boxIndex, curr, (MovingBox)newBox);
        } else {
            return collisionExist(log.boxIndex, curr, (MovingObstacle)newBox);
        }
    }


    private boolean collisionExist(int index, Vertex curr, MovingBox newBox) {

        double width = newBox.getWidth();
        Box oldBox = curr.getMovingBoxes().get(index);

        Rectangle2D rect = new Rectangle2D.Double(
                (oldBox.getPos().getX() + newBox.getPos().getX()) / 2,
                (oldBox.getPos().getY() + newBox.getPos().getY()) / 2,
                width + abs(oldBox.getPos().getX() - newBox.getPos().getX()),
                width + abs(oldBox.getPos().getY() - newBox.getPos().getY())
        );
        return collisionExist(oldBox, curr, rect);
    }

    private boolean collisionExist(int index, Vertex curr, MovingObstacle newBox) {

        double width = newBox.getWidth();
        Box oldBox = curr.getMovingObstacles().get(index);

        Rectangle2D rect = new Rectangle2D.Double(
                (oldBox.getPos().getX() + newBox.getPos().getX()) / 2,
                (oldBox.getPos().getY() + newBox.getPos().getY()) / 2,
                width + abs(oldBox.getPos().getX() - newBox.getPos().getX()),
                width + abs(oldBox.getPos().getY() - newBox.getPos().getY())
        );
        return collisionExist(oldBox, curr, rect);
    }

    // old excluded
    private boolean collisionExist(Box old, Vertex curr, Rectangle2D rect) {
        List<Box> movingBoxes = curr.getMovingBoxes();
        List<Box> movingObs = curr.getMovingObstacles();
        List<StaticObstacle> staticObstacles = ps.getStaticObstacles();

        for(Box obs : movingObs) {
            if(obs.equals(old)) continue; // exclude the old box
            if(rect.intersects(obs.getRect())) {
                return true;
            }
        }

        for(Box obs : movingBoxes) {
            if(obs.equals(old)) continue;
            if(rect.intersects(obs.getRect())) {
                return true;
            }
        }

        for(StaticObstacle obs : staticObstacles) {
            if(rect.intersects(obs.getRect())) {
                return true;
            }
        }
        return false;
    }
}
