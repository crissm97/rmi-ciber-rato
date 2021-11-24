#  (c) Luis Seabra Lopes
# Edited by Bruno Aguiar 80177, José Moreira 79671.
#  

from abc import ABC, abstractmethod

# Dominios de pesquisa
# Permitem calcular
# as accoes possiveis em cada estado, etc
class SearchDomain(ABC):

    # construtor
    @abstractmethod
    def __init__(self):
        pass

    # lista de accoes possiveis num estado
    @abstractmethod
    def actions(self, state):
        pass

    # resultado de uma accao num estado, ou seja, o estado seguinte
    @abstractmethod
    def result(self, state, action):
        pass

    # custo de uma accao num estado
    @abstractmethod
    def cost(self, state, action):
        pass

    # custo estimado de chegar de um estado a outro
    @abstractmethod
    def heuristic(self, state, goal_state):
        pass

# Problemas concretos a resolver
# dentro de um determinado dominio
class SearchProblem: 
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal
    def goal_test(self, state):
        return state == self.goal

# Nos de uma arvore de pesquisa
class SearchNode:
    def __init__(self, state, parent,depth, cost, heuristic):
        self.state = state
        self.parent = parent
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic

    def in_parent(self, state):
        if self.parent == None:
            return False

        return self.parent.state == state or self.parent.in_parent(state)

    def __str__(self):
        return f"no({self.state}, {self.parent}, {self.depth})"
    def __repr__(self):
        return str(self)

# Arvores de pesquisa
class SearchTree:

    # construtor
    def __init__(self, problem, strategy='a*'):
        self.problem = problem
        self.root = SearchNode(problem.initial, None, 0, 0, self.problem.domain.heuristic(problem.initial, problem.goal))
        self.open_nodes = [self.root]
        self.strategy = strategy
        self.cost = 0
        self.length = 0

    # obter o caminho (sequencia de estados) da raiz ate um no
    def get_path(self,node):
        if node.parent == self.root or node.parent == None: # antes: node.parent == None # se der algum erro: filtrar as teclas vazias
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return(path)

    # procurar a solucao
    def search(self):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            self.cost += node.cost
            self.length += 1
            #print(self.problem.goal)
            #print(self.length)
            if self.problem.goal_test(node.state):
                return self.get_path(node), node.cost
            elif self.length >= 2000:
                return self.get_path(node), node.cost, node.depth
            lnewnodes = []
            for action in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state, action)
                if not node.in_parent(newstate) and self.length < 2000: #ou node.depth < limit:
                    lnewnodes += [SearchNode(newstate,node, node.depth+1, node.cost+self.problem.domain.cost(node.state, action), self.problem.domain.heuristic(newstate, self.problem.goal))]

            self.add_to_open(lnewnodes)
        return None

    def add_to_open(self, lnewnodes):
        if self.strategy == 'breadth':
            self.open_nodes.extend(lnewnodes)
        elif self.strategy == 'depth':
            self.open_nodes[:0] = lnewnodes
        elif self.strategy == 'uniform':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.cost)
        elif self.strategy == 'greedy':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.heuristic)
        elif self.strategy == 'a*':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.heuristic + node.cost)

