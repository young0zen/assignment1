package solver;

import org.junit.Before;
import org.junit.Test;
import problem.ProblemSpec;

import java.io.IOException;

import static org.junit.Assert.*;

public class RoadmapTest {
    Roadmap rm;
    static final int MAX_ITER = 10000;
    @Before
    public void setUp() throws Exception {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input1.txt");
            ps.loadSolution("output1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }
        System.out.println("Finished loading!");
        rm = new RoadmapForBox(ps);
    }

    @Test
    public void addNewVertex() {
        try {
            for(int i = 0; i < MAX_ITER; i++) {
                if(rm.addNewVertex(rm.newVertex())) {
                    System.out.println("success!");
                    break;
                }
            }
            System.out.println("No solution");
            return;
        } catch (InsertFailedException e) {
            e.printStackTrace();
            fail();
        }
    }

    @Test
    public void newVertex() {

    }

    @Test
    public void printTrace() {
    }
}