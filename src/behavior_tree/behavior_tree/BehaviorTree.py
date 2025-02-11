from abc import ABC, abstractmethod
import random

# for tree visualization
import networkx as nx
import matplotlib.pyplot as plt


class Node(ABC):
    def __init__(self, name):
        self.name = name
        self.parent = None 
        self.child = None 

    def display(self, save_path=''):
        # display function shows tree as a directed graph (pass root node)
        
        pass

    def traverse(self):
        # traverse the tree depth-first
        print(self.name)

        if self.child:
            self.child.traverse()

    @abstractmethod
    def evaluate(self): # all children are required to implement this function
        pass
    
# Composite Nodes (Can have multiple children)
class Composite(Node):
    def __init__(self, name):
        super().__init__(name)
        self.children = []    

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

    def traverse(self):
        # traverse the tree depth-first
        print(self.name)

        for child in self.children:
            child.traverse()
        
class Sequence(Composite):
    def evaluate(self):
        # runs all children until one returns false
        print(f"Executing Sequence: {self.name}")
        for child in self.children:
            if not child.evaluate():
                return False
        return True

class Selector(Composite):
    def evaluate(self):
        # runs all children until one returns true
        print(f"Executing Selector: {self.name}")
        for child in self.children:
            if child.evaluate():
                return True
        return False
    
# Decorator Nodes (Modify child behavior)    
class Inverter(Node):
    def __init__(self, name, child):
        super().__init__(name)
        self.child = child    
        
    def evaluate(self):
        print(f"Inverting result of: {self.child.name}")
        return not self.child.evaluate()
    
# Condition Nodes (passed condition function must return only True/False)    
class Condition(Node):
    def __init__(self, name, condition):
        super().__init__(name)
        self.condition = condition    
        
    def evaluate(self):
        result = self.condition()
        print(f"Checking Condition: {self.name} -> {result}")
        return result
    
# Action Nodes (Perform actions)    
class Action(Node):
    def __init__(self, name, action):
        super().__init__(name)
        self.action = action    
    def evaluate(self):
        print(f"Executing Action: {self.name}")
        return self.action()# Example Conditions



# example
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
root.traverse()