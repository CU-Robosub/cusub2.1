from abc import ABC, abstractmethod
import random

# for tree visualization
import graphviz


class Node(ABC):
    def __init__(self, name):
        self.name = name
        self.parent = None 
        self.child = None 
        self.node_color = 'grey'

    def display(self, save_name='tree'):
        # display function shows tree as a directed graph

        # graph stores the tree structure
        G = graphviz.Digraph()
        G.node(self.name, style='filled,rounded', fillcolor=self.node_color, color ='black', shape ='box')

        # helper function used to collect all of the edge connections when traversing the tree
        def graphEdges(node, graph): # edges is a list of edges
            if node.parent:
                print(f'[{node.parent.name} -> {node.name}]')
                graph.node(node.name, style='filled,rounded', fillcolor=node.node_color, color ='black', shape ='box')
                graph.edge(node.parent.name, node.name)

        # get all edges by traversing the tree and use them to create a directed graph
        self.traverse(lambda node : graphEdges(node, G))

        # Create legend as a subgraph
        with G.subgraph(name="cluster_legend") as legend:
            legend.attr(label="Legend", fontsize="16", style="filled", color="lightgrey")
            
            # Define legend items
            legend.node("Composite", label="Composite Node", style="filled,rounded", fillcolor="bisque4", shape="box")
            legend.node("Sequence", label="Sequence Node", style="filled,rounded", fillcolor="firebrick", shape="box")
            legend.node("Selector", label="Selector Node", style="filled,rounded", fillcolor="darkorange", shape="box")
            legend.node("Decorator", label="Inverter Node", style="filled,rounded", fillcolor="crimson", shape="box")
            legend.node("Condition", label="Condition Node", style="filled,rounded", fillcolor="midnightblue", shape="box")
            legend.node("Action", label="Action Node", style="filled,rounded", fillcolor="darkolivegreen", shape="box")
            
            # Arrange legend items using invisible edges
            legend.edge("Composite", "Sequence", style="invis")
            legend.edge("Sequence", "Selector", style="invis")
            legend.edge("Selector", "Decorator", style="invis")
            legend.edge("Decorator", "Condition", style="invis")
            legend.edge("Condition", "Action", style="invis")
        
        # draw the graph with a tree layout
        G.render(save_name, format="png")

    def traverse(self, function):
        # traverse the tree depth-first, executing function at each step
        function(self)

        if self.child:
            self.child.traverse(function)

    @abstractmethod
    def evaluate(self): # all children are required to implement this function
        pass
    
# Composite Nodes (Can have multiple children)
class Composite(Node):
    def __init__(self, name):
        super().__init__(name)
        self.children = []   
        self.node_color = 'bisque4' 

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

    def traverse(self, function):
        # traverse the tree depth-first
        function(self)

        for child in self.children:
            child.traverse(function)
        
class Sequence(Composite):
    def __init__(self, name):
        super().__init__(name)
        self.node_color = 'firebrick'

    def evaluate(self):
        # runs all children until one returns false
        print(f"Executing Sequence: {self.name}")
        for child in self.children:
            if not child.evaluate():
                return False
        return True

class Selector(Composite):
    def __init__(self, name):
        super().__init__(name)
        self.node_color = 'darkorange'

    def evaluate(self):
        # runs all children until one returns true
        print(f"Executing Selector: {self.name}")
        for child in self.children:
            if child.evaluate():
                return True
        return False
    
# Inverter Nodes (Modify child behavior)    
class Inverter(Node):
    def __init__(self, name, child):
        super().__init__(name)
        self.child = child    
        self.node_color = 'crimson'
        
    def evaluate(self):
        print(f"Inverting result of: {self.child.name}")
        return not self.child.evaluate()
    
# Condition Nodes (passed condition function must return only True/False)    
class Condition(Node):
    def __init__(self, name, condition):
        super().__init__(name)
        self.condition = condition   
        self.node_color = 'midnightblue' 
        
    def evaluate(self):
        result = self.condition()
        print(f"Checking Condition: {self.name} -> {result}")
        return result
    
# Action Nodes (Perform actions)    
class Action(Node):
    def __init__(self, name, action):
        super().__init__(name)
        self.action = action    
        self.node_color = 'darkolivegreen'

    def evaluate(self):
        print(f"Executing Action: {self.name}")
        return self.action()# Example Conditions



# example
if __name__=='__main__':
    def enemy_near():
        return random.randint(0,1)
    def low_health():
        return random.randint(0,1)# Example Actions
    def task_move():
        print("Moving...")
        return True
    def task_attack():
        print("Attacking...")
        return True
    def task_retreat():
        print("Retreating...")
        return True
    def task_defend():
        print("Defending...")
        return True
    def task_patrol():
        print("Patrolling...")
        return True# Building the Behavior Tree
    root = Selector("Root Selector")# Sequence for combat
    combat_sequence = Sequence("Combat Sequence")
    combat_sequence.add_child(Condition("Enemy Near?", enemy_near))
    combat_sequence.add_child(Action("Attack", task_attack))# Defensive behavior
    defensive_selector = Selector("Defensive Behavior")
    defensive_selector.add_child(Condition("Low Health?", low_health))
    defensive_selector.add_child(Action("Retreat", task_retreat))
    defensive_selector.add_child(Action("Defend", task_defend))# Fallback patrol if nothing else happens
    patrol_action = Action("Patrol", task_patrol)# Assembling tree
    root.add_child(combat_sequence)  # Tries combat first
    root.add_child(defensive_selector)  # Falls back to defensive actions
    root.add_child(patrol_action)  # If nothing else, patrol# Running the behavior tree
    root.evaluate()
    root.display(save_name='my_Tree')