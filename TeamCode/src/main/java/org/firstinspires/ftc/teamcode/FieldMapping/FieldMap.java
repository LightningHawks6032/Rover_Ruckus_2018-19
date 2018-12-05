package org.firstinspires.ftc.teamcode.FieldMapping;

import java.util.HashMap;
import java.util.Map;

public class FieldMap {
    // Creates a map: each Vector position is mapped to a string identifier
    private Map<String, Vector> map = new HashMap<>();

    public final double FULL_FIELD_LENGTH = 140;
    public final double QUADRANT_LENGTH = FULL_FIELD_LENGTH/2;
    public final double SQUARE_LENGTH = FULL_FIELD_LENGTH/6;
    public final double HALF_SQUARE_LENGTH = SQUARE_LENGTH/2;
    public final double MID_QUADRANT = QUADRANT_LENGTH/2;

    // Constructor generates the map
    public FieldMap() {
        generate();
    }

    private void generate() {

        // Quadrant 2
        // Minerals
        addElement(2, "Left Mineral", new Vector(-2*SQUARE_LENGTH, SQUARE_LENGTH));
        addElement(2, "Middle Mineral", new Vector(-MID_QUADRANT, MID_QUADRANT));
        addElement(2, "Right Mineral", new Vector(-SQUARE_LENGTH, 2*SQUARE_LENGTH));
        // Blue Depot
        addElement(2, "Blue Depot", new Vector(-5*HALF_SQUARE_LENGTH,5*HALF_SQUARE_LENGTH));

        // Quadrant 3
        // Minerals
        addElement(3, "Left Mineral", new Vector(-SQUARE_LENGTH, -2*SQUARE_LENGTH));
        addElement(3, "Middle Mineral", new Vector(-MID_QUADRANT, -MID_QUADRANT));
        addElement(3, "Right Mineral", new Vector(-2*SQUARE_LENGTH, -SQUARE_LENGTH));
        // Crater Position
        addElement(3,"Crater Left Edge", new Vector(-SQUARE_LENGTH,-5*HALF_SQUARE_LENGTH));
        addElement(3,"Crater Right Edge", new Vector(-5*HALF_SQUARE_LENGTH,-SQUARE_LENGTH));
        addElement(3,"Crater Center", new Vector(-5*HALF_SQUARE_LENGTH,-5*HALF_SQUARE_LENGTH));

        // Quadrant 4
        // Minerals
        addElement(4, "Left Mineral", new Vector(2*SQUARE_LENGTH, -SQUARE_LENGTH));
        addElement(4, "Middle Mineral", new Vector(MID_QUADRANT, -MID_QUADRANT));
        addElement(4, "Right Mineral", new Vector(SQUARE_LENGTH, -2*SQUARE_LENGTH));
        // Red Depot
        addElement(4, "Red Depot", new Vector(5*HALF_SQUARE_LENGTH,-5*HALF_SQUARE_LENGTH));

        // Quadrant 1
        // Minerals
        addElement(1, "Left Mineral", new Vector(SQUARE_LENGTH, 2*SQUARE_LENGTH));
        addElement(1, "Middle Mineral", new Vector(MID_QUADRANT, MID_QUADRANT));
        addElement(1, "Right Mineral", new Vector(2*SQUARE_LENGTH, SQUARE_LENGTH));
        // Crater Positions
        addElement(1,"Crater Left Edge", new Vector(SQUARE_LENGTH,5*HALF_SQUARE_LENGTH));
        addElement(1,"Crater Right Edge", new Vector(5*HALF_SQUARE_LENGTH,SQUARE_LENGTH));
        addElement(1,"Crater Center", new Vector(5*HALF_SQUARE_LENGTH,5*HALF_SQUARE_LENGTH));

        // Nav Targets
        addElement(0, "Blue Rover", new Vector(0,QUADRANT_LENGTH));
        addElement(0, "Front Craters", new Vector(QUADRANT_LENGTH,0));
        addElement(0, "Back Space", new Vector(-QUADRANT_LENGTH,0));
        addElement(0, "Red Footprint", new Vector(0,-QUADRANT_LENGTH));
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
