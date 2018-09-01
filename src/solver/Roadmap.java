package solver;

import problem.ProblemSpec;

import java.util.List;

public interface Roadmap {

    /**
     * Add the sample or derived Vertex to the current map
     * @param sample the Vertex to be added
     * @return true if the map can now reach the goal
     */
    public boolean addNewVertex(Vertex sample) throws InsertFailedException;

    /**
     * Use some kind of sample strategy to find a new Vertex
     * @return the new Vertex
     */
    public Vertex newVertex();

    public void printTrace();
}
