package org.firstinspires.ftc.teamcode.FieldMapping;

import java.util.HashMap;
import java.util.Map;

public class FieldMap {
    // Creates a map: each Vector position is mapped to a string identifier
    private Map<String, Vector> map = new HashMap<>();

    // Constructor generates the map
    public FieldMap() {
        generate();
    }

    private void generate() {
        // Quadrant 1 Minerals
        addElement(1, "Left Mineral", new Vector(23.33, 46.67));
        addElement(1, "Middle Mineral", new Vector(35, 35));
        addElement(1, "Right Mineral", new Vector(46.67, 23.33));

        // Quadrant 2 Minerals
        addElement(2, "Left Mineral", new Vector(-46.67, 23.33));
        addElement(2, "Middle Mineral", new Vector(-35, 35));
        addElement(2, "Right Mineral", new Vector(-23.33, 46.67));

        // Quadrant 3 Minerals
        addElement(3, "Left Mineral", new Vector(-23.33, -46.67));
        addElement(3, "Middle Mineral", new Vector(-35, -35));
        addElement(3, "Right Mineral", new Vector(-46.67, -23.33));

        // Quadrant 4 Minerals
        addElement(4, "Left Mineral", new Vector(46.67, -23.33));
        addElement(4, "Middle Mineral", new Vector(35, -35));
        addElement(4, "Right Mineral", new Vector(23.33, -46.67));
    }

    // Accesses the position of a specific element based on quadrant and name
    public Vector get(int quadrant, String element) {
        return map.get(quadrant + ": " + element);
    }

    // Adds an element to the field map with quadrant, name, and vector position
    private void addElement(int quadrant, String element, Vector position) {
        map.put(quadrant + ": " + element, position);
    }
}
