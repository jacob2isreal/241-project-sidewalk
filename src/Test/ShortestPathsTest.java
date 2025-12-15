import static org.junit.Assert.*;

import org.junit.Test;

import java.net.URL;
import java.io.FileNotFoundException;

import java.util.LinkedList;

public class ShortestPathsTest {


    /* Returns the Graph loaded from the file with filename fn. */
    private Graph loadBasicGraph(String fn) {
        Graph result = null;
        try {
          result = ShortestPaths.parseGraph("basic", fn);
        } catch (FileNotFoundException e) {
          fail("Could not find graph " + fn);
        }
        return result;
    }

    /** Dummy test case demonstrating syntax to create a graph from scratch.
     * TODO Write your own tests below. */
    @Test
    public void test00Nothing() {
        Graph g = new Graph();
        Node a = g.getNode("A");
        Node b = g.getNode("B");
        g.addEdge(a, b, 1);

        // sample assertion statements:
        assertTrue(true);
        assertEquals(2+2, 4);
    }

    /** Minimal test case to check the path from A to B in Simple0.txt */
    @Test
    public void test01Simple0() {
        Graph g = loadBasicGraph("data/Simple0.txt");
        g.report();
        ShortestPaths sp = new ShortestPaths();
        Node a = g.getNode("A");
        sp.compute(a);
        Node b = g.getNode("B");
        LinkedList<Node> abPath = sp.shortestPath(b);
        assertEquals(abPath.size(), 2);
        assertEquals(abPath.getFirst(), a);
        assertEquals(abPath.getLast(),  b);
        assertEquals(sp.shortestPathLength(b), 1.0, 1e-6);
    }
    @Test
    public void test02ShortestPath() {
	Graph g = loadBasicGraph("data/Sample1.txt");
	ShortestPaths sp = new ShortestPaths();
	Node a = g.getNode("A");
	sp.compute(a);
	Node e = g.getNode("E");
	LinkedList<Node> aePath = sp.shortestPath(e);
	assertEquals(aePath.size(), 2);
	assertEquals(aePath.getFirst(), a);
	assertEquals(aePath.getLast(), e);
	assertEquals(sp.shortestPathLength(e), 3.0, 1e-6);
    }
    @Test
    public void test03UnreachablePath() {
	Graph g = loadBasicGraph("data/Sample2.txt");
	ShortestPaths sp = new ShortestPaths();
	Node a = g.getNode("A");
	sp.compute(a);
	Node g2 = g.getNode("G");
	LinkedList<Node> agPath = sp.shortestPath(g2);
	assertEquals(agPath, null);
    }
    /* Pro tip: unless you include @Test on the line above your method header,
     * JUnit will not run it! This gets me every time. */
}
