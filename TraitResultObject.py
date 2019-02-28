class TraitResult:
	
    def __init__(self):
        self.traits = 0
        self.result = 0
        self.minDistanceVec = []
        self.collisionVec = []


    def assign (self,x,y,z,a):
        self.traits = x;
        self.result = y;
        self.minDistanceVec = z
        self.collisionVec = a