class TraitResult:
	
    def __init__(self):
        self.traits = 0
        self.result = 0
        self.collisions = 0
        self.resultVec = []


    def assign (self,x,y,z,a):
        self.traits = x
        self.result = y
        self.collisions = z
        self.resultVec = a
