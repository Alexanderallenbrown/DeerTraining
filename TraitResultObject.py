class TraitResult:
	
    def __init__(self):
        self.traits = 0
        self.result = 0
        self.minDistanceVec = []


    def assign (self,x,y,z):
        self.traits = x;
        self.result = y;
        self.minDistanceVec = z