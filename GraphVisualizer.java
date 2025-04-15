// GraphVisualizer.java
// A multi-threaded, interactive graph visualization application
// Features:
// - Dynamic graph generation with customizable algorithms
// - Real-time visualization with physics simulation
// - Multi-threaded processing for smooth performance
// - Interactive manipulation of nodes and edges
// - Algorithm visualization (pathfinding, MST, etc.)

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.stream.Collectors;
import javax.swing.Timer;

public class GraphVisualizer extends JFrame {
    private final GraphPanel graphPanel;
    private final ControlPanel controlPanel;
    private final Graph graph;
    private final ExecutorService executorService;

    public GraphVisualizer() {
        super("Advanced Graph Visualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(1200, 800);
        setLayout(new BorderLayout());

        // Initialize the graph
        graph = new Graph();
        
        // Create executor service for multi-threading
        executorService = Executors.newFixedThreadPool(
            Runtime.getRuntime().availableProcessors()
        );

        // Initialize UI components
        graphPanel = new GraphPanel(graph, executorService);
        controlPanel = new ControlPanel(graph, graphPanel);
        
        // Add components to frame
        add(graphPanel, BorderLayout.CENTER);
        add(controlPanel, BorderLayout.EAST);
        
        // Start the physics simulation
        startSimulation();
        
        setLocationRelativeTo(null);
        setVisible(true);
    }
    
    private void startSimulation() {
        Timer timer = new Timer(16, e -> graphPanel.repaint());
        timer.start();
        
        // Run physics simulation in separate thread
        executorService.submit(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                graph.updatePhysics();
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
    }
    
    @Override
    public void dispose() {
        executorService.shutdownNow();
        super.dispose();
    }

    public static void main(String[] args) {
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        SwingUtilities.invokeLater(GraphVisualizer::new);
    }
}

// Graph data structure
class Graph {
    private final List<Node> nodes;
    private final List<Edge> edges;
    private final Random random;
    private final Object lock;
    
    public Graph() {
        nodes = Collections.synchronizedList(new ArrayList<>());
        edges = Collections.synchronizedList(new ArrayList<>());
        random = new Random();
        lock = new Object();
    }
    
    public void generateRandomGraph(int nodeCount, double edgeProbability) {
        synchronized (lock) {
            clearGraph();
            
            // Create nodes
            for (int i = 0; i < nodeCount; i++) {
                addNode(new Node(
                    random.nextInt(800) + 100,
                    random.nextInt(500) + 100,
                    "Node " + i
                ));
            }
            
            // Create edges
            for (int i = 0; i < nodes.size(); i++) {
                for (int j = i + 1; j < nodes.size(); j++) {
                    if (random.nextDouble() < edgeProbability) {
                        addEdge(new Edge(nodes.get(i), nodes.get(j)));
                    }
                }
            }
        }
    }
    
    public void generateGridGraph(int rows, int cols) {
        synchronized (lock) {
            clearGraph();
            
            // Create nodes in a grid
            Node[][] nodeGrid = new Node[rows][cols];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    Node node = new Node(
                        100 + j * 100,
                        100 + i * 100,
                        "Node " + (i * cols + j)
                    );
                    nodeGrid[i][j] = node;
                    addNode(node);
                }
            }
            
            // Create edges
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (j < cols - 1) {
                        addEdge(new Edge(nodeGrid[i][j], nodeGrid[i][j + 1]));
                    }
                    if (i < rows - 1) {
                        addEdge(new Edge(nodeGrid[i][j], nodeGrid[i + 1][j]));
                    }
                }
            }
        }
    }
    
    public void generateCircularGraph(int nodeCount) {
        synchronized (lock) {
            clearGraph();
            
            double radius = 300;
            double centerX = 500;
            double centerY = 350;
            
            // Create nodes in a circle
            for (int i = 0; i < nodeCount; i++) {
                double angle = 2 * Math.PI * i / nodeCount;
                int x = (int) (centerX + radius * Math.cos(angle));
                int y = (int) (centerY + radius * Math.sin(angle));
                addNode(new Node(x, y, "Node " + i));
            }
            
            // Create edges around the circle
            for (int i = 0; i < nodeCount; i++) {
                addEdge(new Edge(nodes.get(i), nodes.get((i + 1) % nodeCount)));
            }
        }
    }
    
    public void generateScaleFreeGraph(int nodeCount) {
        synchronized (lock) {
            clearGraph();
            
            // Start with a small complete graph
            for (int i = 0; i < 3; i++) {
                addNode(new Node(
                    random.nextInt(800) + 100,
                    random.nextInt(500) + 100,
                    "Node " + i
                ));
            }
            
            // Connect initial nodes
            for (int i = 0; i < nodes.size(); i++) {
                for (int j = i + 1; j < nodes.size(); j++) {
                    addEdge(new Edge(nodes.get(i), nodes.get(j)));
                }
            }
            
            // Add remaining nodes with preferential attachment
            for (int i = 3; i < nodeCount; i++) {
                Node newNode = new Node(
                    random.nextInt(800) + 100,
                    random.nextInt(500) + 100,
                    "Node " + i
                );
                addNode(newNode);
                
                // Preferential attachment
                for (int j = 0; j < 2; j++) {
                    Node target = getPreferentialNode();
                    if (target != null) {
                        addEdge(new Edge(newNode, target));
                    }
                }
            }
        }
    }
    
    private Node getPreferentialNode() {
        // Count the degree of each node
        Map<Node, Integer> degrees = new HashMap<>();
        for (Node node : nodes) {
            degrees.put(node, 0);
        }
        
        for (Edge edge : edges) {
            degrees.put(edge.getSource(), degrees.get(edge.getSource()) + 1);
            degrees.put(edge.getTarget(), degrees.get(edge.getTarget()) + 1);
        }
        
        // Calculate total degree
        int totalDegree = degrees.values().stream().mapToInt(Integer::intValue).sum();
        if (totalDegree == 0) return nodes.get(random.nextInt(nodes.size()));
        
        // Select node with probability proportional to degree
        int randomValue = random.nextInt(totalDegree);
        int cumulativeDegree = 0;
        
        for (Map.Entry<Node, Integer> entry : degrees.entrySet()) {
            cumulativeDegree += entry.getValue();
            if (cumulativeDegree > randomValue) {
                return entry.getKey();
            }
        }
        
        return nodes.get(random.nextInt(nodes.size()));
    }
    
    public void clearGraph() {
        synchronized (lock) {
            nodes.clear();
            edges.clear();
        }
    }
    
    public void addNode(Node node) {
        synchronized (lock) {
            nodes.add(node);
        }
    }
    
    public void addEdge(Edge edge) {
        synchronized (lock) {
            edges.add(edge);
        }
    }
    
    public void updatePhysics() {
        synchronized (lock) {
            // Apply repulsive forces between all nodes
            for (int i = 0; i < nodes.size(); i++) {
                for (int j = i + 1; j < nodes.size(); j++) {
                    Node n1 = nodes.get(i);
                    Node n2 = nodes.get(j);
                    
                    double dx = n2.getX() - n1.getX();
                    double dy = n2.getY() - n1.getY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    
                    if (distance < 1) distance = 1;
                    
                    // Repulsive force - inverse square law
                    double force = 1500.0 / (distance * distance);
                    
                    // Normalize direction vector
                    double normalizedDx = dx / distance;
                    double normalizedDy = dy / distance;
                    
                    // Apply force
                    n1.addForce(-normalizedDx * force, -normalizedDy * force);
                    n2.addForce(normalizedDx * force, normalizedDy * force);
                }
            }
            
            // Apply attractive forces along edges
            for (Edge edge : edges) {
                Node source = edge.getSource();
                Node target = edge.getTarget();
                
                double dx = target.getX() - source.getX();
                double dy = target.getY() - source.getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance < 1) distance = 1;
                
                // Spring force - proportional to distance
                double force = 0.05 * distance;
                
                // Normalize direction vector
                double normalizedDx = dx / distance;
                double normalizedDy = dy / distance;
                
                // Apply force
                source.addForce(normalizedDx * force, normalizedDy * force);
                target.addForce(-normalizedDx * force, -normalizedDy * force);
            }
            
            // Update positions
            for (Node node : nodes) {
                node.update();
            }
        }
    }
    
    public List<Node> getNodes() {
        synchronized (lock) {
            return new ArrayList<>(nodes);
        }
    }
    
    public List<Edge> getEdges() {
        synchronized (lock) {
            return new ArrayList<>(edges);
        }
    }
    
    public void runDijkstra(Node start, Node end) {
        if (start == null || end == null) return;
        
        // Reset all nodes
        for (Node node : nodes) {
            node.setDistance(Double.POSITIVE_INFINITY);
            node.setPrevious(null);
            node.setVisited(false);
        }
        
        start.setDistance(0);
        
        // Create priority queue
        PriorityQueue<Node> queue = new PriorityQueue<>(
            Comparator.comparingDouble(Node::getDistance)
        );
        queue.add(start);
        
        while (!queue.isEmpty()) {
            Node current = queue.poll();
            if (current.isVisited()) continue;
            
            current.setVisited(true);
            
            // If we've reached the target, we can stop
            if (current == end) break;
            
            // Find all neighbors
            Set<Node> neighbors = edges.stream()
                .filter(e -> e.getSource() == current || e.getTarget() == current)
                .map(e -> e.getSource() == current ? e.getTarget() : e.getSource())
                .collect(Collectors.toSet());
            
            for (Node neighbor : neighbors) {
                if (neighbor.isVisited()) continue;
                
                double edgeWeight = 1.0; // Using uniform weights for simplicity
                double newDistance = current.getDistance() + edgeWeight;
                
                if (newDistance < neighbor.getDistance()) {
                    neighbor.setDistance(newDistance);
                    neighbor.setPrevious(current);
                    queue.add(neighbor);
                }
            }
        }
        
        // Mark the shortest path
        Node current = end;
        while (current != null && current.getPrevious() != null) {
            // Find the edge between current and its previous
            for (Edge edge : edges) {
                if ((edge.getSource() == current && edge.getTarget() == current.getPrevious()) ||
                    (edge.getTarget() == current && edge.getSource() == current.getPrevious())) {
                    edge.setHighlighted(true);
                }
            }
            current = current.getPrevious();
        }
    }
}

class Node {
    private double x, y;
    private double velocityX, velocityY;
    private double forceX, forceY;
    private String label;
    private double distance;
    private Node previous;
    private boolean visited;
    private boolean highlighted;
    private static final double DAMPING = 0.8;
    
    public Node(double x, double y, String label) {
        this.x = x;
        this.y = y;
        this.label = label;
        this.velocityX = 0;
        this.velocityY = 0;
        this.forceX = 0;
        this.forceY = 0;
        this.distance = Double.POSITIVE_INFINITY;
        this.visited = false;
        this.highlighted = false;
    }
    
    public void addForce(double fx, double fy) {
        this.forceX += fx;
        this.forceY += fy;
    }
    
    public void update() {
        // Update velocity using force
        velocityX = (velocityX + forceX) * DAMPING;
        velocityY = (velocityY + forceY) * DAMPING;
        
        // Update position using velocity
        x += velocityX;
        y += velocityY;
        
        // Constrain to visible area with some padding
        x = Math.max(50, Math.min(1100, x));
        y = Math.max(50, Math.min(700, y));
        
        // Reset forces
        forceX = 0;
        forceY = 0;
    }
    
    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }
    
    public String getLabel() {
        return label;
    }
    
    public void setDistance(double distance) {
        this.distance = distance;
    }
    
    public double getDistance() {
        return distance;
    }
    
    public void setPrevious(Node previous) {
        this.previous = previous;
    }
    
    public Node getPrevious() {
        return previous;
    }
    
    public void setVisited(boolean visited) {
        this.visited = visited;
    }
    
    public boolean isVisited() {
        return visited;
    }
    
    public void setHighlighted(boolean highlighted) {
        this.highlighted = highlighted;
    }
    
    public boolean isHighlighted() {
        return highlighted;
    }
}

class Edge {
    private final Node source;
    private final Node target;
    private boolean highlighted;
    
    public Edge(Node source, Node target) {
        this.source = source;
        this.target = target;
        this.highlighted = false;
    }
    
    public Node getSource() {
        return source;
    }
    
    public Node getTarget() {
        return target;
    }
    
    public void setHighlighted(boolean highlighted) {
        this.highlighted = highlighted;
    }
    
    public boolean isHighlighted() {
        return highlighted;
    }
}

class GraphPanel extends JPanel {
    private final Graph graph;
    private final ExecutorService executorService;
    private Node selectedNode;
    private Node hoveredNode;
    private Node pathStartNode;
    private Node pathEndNode;
    
    public GraphPanel(Graph graph, ExecutorService executorService) {
        this.graph = graph;
        this.executorService = executorService;
        this.selectedNode = null;
        this.hoveredNode = null;
        this.pathStartNode = null;
        this.pathEndNode = null;
        
        setBackground(Color.WHITE);
        
        // Add mouse listeners for interaction
        addMouseListeners();
    }
    
    private void addMouseListeners() {
        MouseAdapter mouseAdapter = new MouseAdapter() {
            private int lastX, lastY;
            
            @Override
            public void mousePressed(MouseEvent e) {
                lastX = e.getX();
                lastY = e.getY();
                
                // Find clicked node if any
                Node clickedNode = findNodeAt(e.getX(), e.getY());
                
                if (e.getButton() == MouseEvent.BUTTON1) {
                    selectedNode = clickedNode;
                    
                    // If shift is pressed, set as path start node
                    if (e.isShiftDown() && clickedNode != null) {
                        pathStartNode = clickedNode;
                        pathStartNode.setHighlighted(true);
                        
                        // If we also have an end node, compute path
                        if (pathEndNode != null) {
                            executorService.submit(() -> {
                                // Reset previous highlights
                                for (Edge edge : graph.getEdges()) {
                                    edge.setHighlighted(false);
                                }
                                
                                graph.runDijkstra(pathStartNode, pathEndNode);
                                repaint();
                            });
                        }
                    }
                } else if (e.getButton() == MouseEvent.BUTTON3 && clickedNode != null) {
                    // Right click - set as path end node
                    pathEndNode = clickedNode;
                    pathEndNode.setHighlighted(true);
                    
                    // If we also have a start node, compute path
                    if (pathStartNode != null) {
                        executorService.submit(() -> {
                            // Reset previous highlights
                            for (Edge edge : graph.getEdges()) {
                                edge.setHighlighted(false);
                            }
                            
                            graph.runDijkstra(pathStartNode, pathEndNode);
                            repaint();
                        });
                    }
                }
                
                repaint();
            }
            
            @Override
            public void mouseDragged(MouseEvent e) {
                if (selectedNode != null) {
                    // Move selected node
                    double dx = e.getX() - lastX;
                    double dy = e.getY() - lastY;
                    
                    selectedNode.addForce(dx * 0.5, dy * 0.5);
                    
                    lastX = e.getX();
                    lastY = e.getY();
                    repaint();
                }
            }
            
            @Override
            public void mouseReleased(MouseEvent e) {
                selectedNode = null;
                repaint();
            }
            
            @Override
            public void mouseMoved(MouseEvent e) {
                // Update hovered node
                hoveredNode = findNodeAt(e.getX(), e.getY());
                repaint();
            }
        };
        
        addMouseListener(mouseAdapter);
        addMouseMotionListener(mouseAdapter);
    }
    
    private Node findNodeAt(int x, int y) {
        for (Node node : graph.getNodes()) {
            double dx = node.getX() - x;
            double dy = node.getY() - y;
            if (Math.sqrt(dx * dx + dy * dy) <= 15) {
                return node;
            }
        }
        return null;
    }
    
    public void clearPathHighlights() {
        pathStartNode = null;
        pathEndNode = null;
        
        for (Edge edge : graph.getEdges()) {
            edge.setHighlighted(false);
        }
        
        for (Node node : graph.getNodes()) {
            node.setHighlighted(false);
        }
        
        repaint();
    }
    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        
        // Draw edges
        for (Edge edge : graph.getEdges()) {
            if (edge.isHighlighted()) {
                g2d.setStroke(new BasicStroke(3));
                g2d.setColor(new Color(255, 100, 0));
            } else {
                g2d.setStroke(new BasicStroke(1));
                g2d.setColor(Color.GRAY);
            }
            
            g2d.drawLine(
                (int) edge.getSource().getX(),
                (int) edge.getSource().getY(),
                (int) edge.getTarget().getX(),
                (int) edge.getTarget().getY()
            );
        }
        
        // Draw nodes
        for (Node node : graph.getNodes()) {
            // Select color based on state
            if (node == hoveredNode) {
                g2d.setColor(new Color(100, 100, 255));
            } else if (node == pathStartNode) {
                g2d.setColor(new Color(0, 200, 0));
            } else if (node == pathEndNode) {
                g2d.setColor(new Color(200, 0, 0));
            } else if (node.isHighlighted()) {
                g2d.setColor(new Color(255, 150, 0));
            } else {
                g2d.setColor(new Color(50, 150, 200));
            }
            
            g2d.fillOval(
                (int) node.getX() - 10,
                (int) node.getY() - 10,
                20, 20
            );
            
            g2d.setColor(Color.BLACK);
            g2d.drawOval(
                (int) node.getX() - 10,
                (int) node.getY() - 10,
                20, 20
            );
            
            // Draw label
            FontMetrics fm = g2d.getFontMetrics();
            int labelWidth = fm.stringWidth(node.getLabel());
            g2d.drawString(
                node.getLabel(),
                (int) node.getX() - labelWidth / 2,
                (int) node.getY() - 15
            );
        }
    }
}

class ControlPanel extends JPanel {
    private final Graph graph;
    private final GraphPanel graphPanel;
    
    public ControlPanel(Graph graph, GraphPanel graphPanel) {
        this.graph = graph;
        this.graphPanel = graphPanel;
        
        setPreferredSize(new Dimension(250, 800));
        setBorder(BorderFactory.createCompoundBorder(
            BorderFactory.createMatteBorder(0, 1, 0, 0, Color.GRAY),
            BorderFactory.createEmptyBorder(10, 10, 10, 10)
        ));
        
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
        
        // Title
        JLabel titleLabel = new JLabel("Graph Controls");
        titleLabel.setFont(new Font("Arial", Font.BOLD, 16));
        titleLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(titleLabel);
        add(Box.createVerticalStrut(20));
        
        // Graph generation section
        JLabel genLabel = new JLabel("Generate Graph");
        genLabel.setFont(new Font("Arial", Font.BOLD, 14));
        genLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(genLabel);
        add(Box.createVerticalStrut(10));
        
        // Random graph button
        JPanel randomPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        randomPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        JButton randomButton = new JButton("Random Graph");
        JSpinner nodeCountSpinner = new JSpinner(new SpinnerNumberModel(20, 5, 100, 1));
        JLabel densityLabel = new JLabel("Edge Density:");
        JSlider densitySlider = new JSlider(JSlider.HORIZONTAL, 1, 10, 3);
        
        randomButton.addActionListener(e -> {
            int nodeCount = (int) nodeCountSpinner.getValue();
            double density = densitySlider.getValue() / 10.0;
            graph.generateRandomGraph(nodeCount, density);
            graphPanel.clearPathHighlights();
        });
        
        randomPanel.add(randomButton);
        randomPanel.add(new JLabel("Nodes:"));
        randomPanel.add(nodeCountSpinner);
        add(randomPanel);
        
        JPanel densityPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        densityPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        densityPanel.add(densityLabel);
        densityPanel.add(densitySlider);
        add(densityPanel);
        add(Box.createVerticalStrut(10));
        
        // Grid graph button
        JPanel gridPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        gridPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        JButton gridButton = new JButton("Grid Graph");
        JSpinner rowsSpinner = new JSpinner(new SpinnerNumberModel(5, 2, 15, 1));
        JSpinner colsSpinner = new JSpinner(new SpinnerNumberModel(5, 2, 15, 1));
        
        gridButton.addActionListener(e -> {
            int rows = (int) rowsSpinner.getValue();
            int cols = (int) colsSpinner.getValue();
            graph.generateGridGraph(rows, cols);
            graphPanel.clearPathHighlights();
        });
        
        gridPanel.add(gridButton);
        gridPanel.add(new JLabel("Rows:"));
        gridPanel.add(rowsSpinner);
        gridPanel.add(new JLabel("Cols:"));
        gridPanel.add(colsSpinner);
        add(gridPanel);
        add(Box.createVerticalStrut(10));
        
        // Circular graph button
        JPanel circlePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        circlePanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        JButton circleButton = new JButton("Circular Graph");
        JSpinner circleNodeSpinner = new JSpinner(new SpinnerNumberModel(12, 3, 50, 1));
        
        circleButton.addActionListener(e -> {
            int nodeCount = (int) circleNodeSpinner.getValue();
            graph.generateCircularGraph(nodeCount);
            graphPanel.clearPathHighlights();
        });
        
        circlePanel.add(circleButton);
        circlePanel.add(new JLabel("Nodes:"));
        circlePanel.add(circleNodeSpinner);
        add(circlePanel);
        add(Box.createVerticalStrut(10));
        
        // Scale-free graph button
        JPanel scalePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        scalePanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        JButton scaleButton = new JButton("Scale-Free Graph");
        JSpinner scaleNodeSpinner = new JSpinner(new SpinnerNumberModel(30, 5, 100, 1));
        
        scaleButton.addActionListener(e -> {
            int nodeCount = (int) scaleNodeSpinner.getValue();
            graph.generateScaleFreeGraph(nodeCount);
            graphPanel.clearPathHighlights();
        });
        
        scalePanel.add(scaleButton);
        scalePanel.add(new JLabel("Nodes:"));
        scalePanel.add(scaleNodeSpinner);
        add(scalePanel);
        add(Box.createVerticalStrut(20));
        
        // Algorithms section
        JLabel algoLabel = new JLabel("Algorithms");
        algoLabel.setFont(new Font("Arial", Font.BOLD, 14));
        algoLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(algoLabel);
        add(Box.createVerticalStrut(10));
        
        // Reset button
        JButton resetButton = new JButton("Reset Visualization");
        resetButton.setAlignmentX(Component.LEFT_ALIGNMENT);
        resetButton.addActionListener(e -> graphPanel.clearPathHighlights());
        add(resetButton);
        add(Box.createVerticalStrut(10));
        
        // Instructions
        JLabel instructionsLabel = new JLabel("<html>Instructions:<br>" +
            "- Drag nodes to reposition<br>" +
            "- Shift+Click to set path start<br>" +
            "- Right-Click to set path end<br>" +
            "- Path will automatically calculate</html>");
        instructionsLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(instructionsLabel);
        add(Box.createVerticalStrut(20));
        
        // Create initial graph
        graph.generateGridGraph(5, 5);
    }
}