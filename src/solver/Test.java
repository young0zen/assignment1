package solver;

import problem.Box;
import problem.MovingBox;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class Test {
    public static void main(String[] strings) {
        List<Box > boxes = new ArrayList<>();
        boxes.add(new MovingBox(new Point2D.Double(0.0, 0.0), 0.3));

        Box box = boxes.get(0);
        box.pos.setLocation(1, 1);

        System.out.println(boxes.get(0).getPos().getX());
    }
}
