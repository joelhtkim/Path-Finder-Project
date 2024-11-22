#lang dssl2

# Final project: Trip Planner

import cons
import sbox_hash



import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/stack-queue.rkt'
import 'project-lib/binheap.rkt'

let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw posToNum are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let roadSegs
    let poiS
    let posToNum
    let matrix
    let catToPos
    let nameToPos
    let posToName
    let numToPos
    let maxPos
    let maxName
    let nameToCat
    

#   ^ YOUR CODE GOES HERE
    def __init__(self, segs, points):
        self.roadSegs = segs
        self.poiS = points
        self.maxPos = segs.len()*2
        self.maxName = points.len()
        self.posToNum = HashTable(self.maxPos, make_sbox_hash()) 
        self.numToPos = HashTable(self.maxPos, make_sbox_hash()) 
        self.matrix = None 
        self.catToPos = HashTable(self.maxName, make_sbox_hash())
        self.nameToPos = HashTable(self.maxName, make_sbox_hash())
        self.posToName = HashTable(self.maxPos, make_sbox_hash())
        self.nameToCat = HashTable(self.maxName, make_sbox_hash())

# name -> The poi name : "The Empty Bottle"
# pos -> Position: [0,0]
# num -> Vertex number in the adjaceny matrix : 1
# cat -> Category: "food"
       
       
        #loop through segments, and all new positions are placed into the posToNum and numToPos dictionaries
        let count = 0
        for i in range(segs.len()):
            let tempPos1 = [self.roadSegs[i][0],self.roadSegs[i][1]]
            let tempPos2 = [self.roadSegs[i][2],self.roadSegs[i][3]]  
            if (self.posToNum.mem?(tempPos1) == False):
                self.posToNum.put(tempPos1,count)
                self.numToPos.put(count, tempPos1)
                count = count + 1
            if (self.posToNum.mem?(tempPos2) == False):
                self.posToNum.put(tempPos2,count)
                self.numToPos.put(count, tempPos2)
                count = count + 1  
                
        # matrix for distances is made
        self.matrix = WuGraph(self.posToNum.len())
        
        # sets the distances for the matrices
        for i in range(segs.len()):           
            let start = [self.roadSegs[i][0],self.roadSegs[i][1]]
            let finish = [self.roadSegs[i][2],self.roadSegs[i][3]]
            let startVert = self.posToNum.get(start)
            let finishVert = self.posToNum.get(finish)
            
            let distance = int((start[0] - finish[0]) * (start[0] - finish[0]) +  (start[1] - finish[1]) * (start[1] - finish[1])).sqrt()
            
            self.matrix.set_edge(startVert,finishVert,distance) 
            self.matrix.set_edge(finishVert,startVert,distance)      
        
            
        # setting up the category to position dictionary
        # setting up the name to position dictionary
        # setting up the position to name dictionary
             
        let dupCheck = HashTable(self.maxPos, make_sbox_hash())
        for i in range(self.poiS.len()):
            let tempPos = [self.poiS[i][0],self.poiS[i][1]]
            let cat = self.poiS[i][2]
            let name = self.poiS[i][3]
            # creates a linked list to allow for multiple values  
            if(self.nameToCat.mem?(name) == False):
                self.nameToCat.put(name,cat)
            if(self.catToPos.mem?(cat) == False):
                self.catToPos.put(cat,cons(tempPos,None))
            else:
                if(dupCheck.mem?([tempPos,cat])==False):
                    self.catToPos.put(cat,cons(tempPos,self.catToPos.get(cat)))
                    dupCheck.put([tempPos,cat],1)          
            self.nameToPos.put(name,tempPos)
            if(self.posToName.mem?(tempPos) == False):
                self.posToName.put(tempPos,cons(name,None))
            else:
                self.posToName.put(tempPos,cons(name,self.posToName.get(tempPos)))
       
                
    # locate all,
    # takes a catgeory and finds all the positions using the category to positions dictionary
    # loops through the linked list and builds a linked list of those positions   
                     
    def locate_all(self,cat):
        if(self.catToPos.mem?(cat) == False):
            return None
        let locations = None           
        let cur = self.catToPos.get(cat)
        # building the linked list
        while (cur != None):
            let curVal = cur.data
            locations = cons(curVal,locations)
            cur = cur.next 
        return locations
    
        
    
        
        
    # dijkstras algorithm
    def dijkstra(self,  startV):
        let pred = vec(self.matrix.len())
        let done = vec(self.matrix.len())
        let dist = vec(self.matrix.len())
        for i in range(self.matrix.len()):
            pred.put(i, None)
            dist.put(i, inf)
            done.put(i,False)
        
        # main portion of dijkstra's algorithm
        dist.put(startV,0)
        let pq = BinHeap(self.maxPos,lambda a, b: dist.get(a) < dist.get(b))
        pq.insert(startV)
        while(pq.len() != 0):
            let v = pq.find_min()
            pq.remove_min()           
            if done.get(v)==False:
                done.put(v,True)
            let neighborCur = self.matrix.get_adjacent(v)
            while (neighborCur != None):
                let neighbor= neighborCur.data
                let alt = dist.get(v) + self.matrix.get_edge(neighbor,v)
                if alt < dist.get(neighbor):
                    dist.put(neighbor,alt)
                    pred.put(neighbor,v)
                    pq.insert(neighbor)
                neighborCur = neighborCur.next
        return [pred,dist]
        
    # plan route
    # checks to see if the poi exists
    # creates a predecessers, done, and distances vectors
    # uses dijkstras algorithm to find the distances and predecssors for the start to the other locations  
    # then follows the path backwards from the desired location to the start
    # keeps track of this route and returns it as a linked list   
    def plan_route(self,lat,lon, poi):
        if (self.nameToPos.mem?(poi) == False):
            return None
        let start = [lat,lon]
        if (self.nameToPos.get(poi) == start):
            return cons(start,None)
        let startV = self.posToNum.get(start)
        let predsAndDist = self.dijkstra(startV)
        let pred = predsAndDist[0]
        let dist = predsAndDist[1]
        # loop follows the path backwards using the table of predecessors
        let finishP = self.nameToPos.get(poi)
        let finishV = self.posToNum.get(finishP)
        let cur = finishV
        let path = None
        if (pred.get(cur) == None):
            return path
        while (cur != startV):
            let curP = self.numToPos.get(cur)
            path = cons(curP,path)
            cur = pred.get(cur)
        path = cons(start,path)
        return path
    
        
    # find nearby
    # uses locate all to find all positions with the desired category
    # loops through the positions and uses path_route to find the shortest path from the start to that position
    # then loops through the shortest path to calculate the total distance and stores the (cons) position and total distance to travel to that position in a minheap
    # Then, finds the lowest values of the minheap and builds the linked list of those locations followign the limit
    
    def find_nearby(self, lat, lon,cat,limit):
        if(self.catToPos.mem?(cat) == False):
            return None
        let startP = [lat,lon]
        let startV = self.posToNum.get(startP)
        let allDist = BinHeap(self.matrix.len(), λ x, y: x.data < y.data)
        let allLocs = self.locate_all(cat)
        
        #looping throught the positions from locate_all
        let cur1 = allLocs
        while (cur1 != None):
            let valP = cur1.data
            let valV = self.posToNum.get(valP)
            let valN = self.posToName.get(valP).data

            # uses dijkstra to find shortest path to destination
            let dij = self.dijkstra(startV)
            let distance = dij[1]
            let disTemp = cons(distance[valV],cons(valP,None))
            if (disTemp.data != inf):
                allDist.insert(disTemp)
            cur1 = cur1.next
        
        # finding closest places               
        let nearest = None
        for i in range(limit):
            if (allDist.len() == 0):
                return nearest
            let nearP = allDist.find_min().next.data
            let nearN = self.posToName.get(nearP).data
            let nearNList = self.posToName.get(nearP)
            let cur = nearNList
            let curNearName = cur.data
            while(cur != None):
                curNearName = cur.data
                if (self.nameToCat.get(curNearName) == cat):
                    nearN = curNearName 
                    break
                cur = cur.next
         
            let tempVec = [nearP[0],nearP[1],cat,nearN]
            if nearest == None:
                nearest = cons(tempVec, None)
            else:
                nearest = cons(tempVec,nearest)
            nearP= allDist.remove_min()
        return nearest
        
            
            
            
                
            
        
    
        
           
            
            
        
        
        


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])
def joel_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0], [1,0, 1,2], [1,2,3,10], [3,10,2,1], [2,1,5,5], [1,0,5,5],[10,10,11,11],[9,9,10,10]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"],
                        [5,5, "place", "Home"],
                        [11,11,"food","Jail"]])
def joel_example2():
    return TripPlanner([[0,0, 0,1], [0,0,2,2], [0,1,3,3], [0,0,10,10]],
                        [[0,1,"food", "pasta"],
                        [2,2,"food", "pizza"],
                        [3,3,"food", "chips"],
                        [10,10, "food", "cake"]])
                        
def joel_example3():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0], [1,0, 1,2], [1,2,3,10], [3,10,2,1], [2,1,5,5], [1,0,5,5],[10,10,11,11],[9,9,10,10]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"],
                        [5,5, "place", "Home"],
                        [11,11,"food","Jail"],
                        [0,1,"bar","Kelly"],
                        [0,1,"bar","Melly"]])
                         
test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)

test 'joel test plan route':
   assert joel_example().plan_route(0, 0, "Home") == \
       cons([0,0], cons([1,0], cons([5,5],None)))
   # Unreachable
   assert joel_example().plan_route(0, 0, "Jail") == None
   # start is the destination
   assert joel_example().plan_route(0,0,"The Empty Bottle") == cons([0,0],None)
   # destination doesn't exist
   assert joel_example().plan_route(0,0, "Not real loc") == None
   
   # start doesn't exist
   #assert joel_example().plan_route(15,19, "Not real start") == None
   

 
test 'joel test locate all':
   assert joel_example2().locate_all("food") == cons([0,1],cons([2,2], cons([3,3], cons([10,10],None))))
   assert joel_example().locate_all("food") == cons([0,1],cons([11,11],None))
   # category doesn't exist
   assert joel_example().locate_all("dog") == None
   assert joel_example().locate_all("bar") == cons([0,0],None)
   # duplicate location
   assert joel_example3().locate_all("bar") == cons([0,0],cons([0,1],None))
   
test 'joel test4 find nearby':
   # lim 1
   assert joel_example2().find_nearby(0, 0, "food",1) == cons([0,1,"food", "pasta"], None)
   # lim 2
   assert joel_example2().find_nearby(0, 0, "food",2) == cons([2,2,"food", "pizza"], cons([0,1,"food", "pasta"],None))
   # lim 3
   assert joel_example2().find_nearby(0, 0, "food",3) == cons([3,3,"food", "chips"],cons([2,2,"food", "pizza"], cons([0,1,"food", "pasta"],None)))
   # limit bigger than total
   assert joel_example2().find_nearby(0, 0, "food",10) == cons([10,10,"food", "cake"],cons([3,3,"food", "chips"],cons([2,2,"food", "pizza"], cons([0,1,"food", "pasta"],None))))
   # cat doesn't exist
   assert joel_example2().find_nearby(0,0,"dog",3) == None
   # nearest is the same as start
   assert joel_example().find_nearby(5,5,"place",1) == cons([5,5,"place","Home"],None)
   
test 'grading':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
    let result = tp.plan_route(0, 0, 'Old North Church')
    assert Cons.to_vec(result) == [[0, 0], [2, 1], [6, 0]]
    
test 'grading2':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.find_nearby(0, 0, 'bank', 1)
    assert (Cons.to_vec(result)) \
      == [[1, 0, 'bank', 'Union']]
test 'g3':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(0, 0, 'barber', 1)
    assert (Cons.to_vec(result)) \
      == [[3, 0, 'barber', 'Tony']]
  
test 'g4':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert (Cons.to_vec(result)) \
      == []

test 'g5':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert (Cons.to_vec(result)) \
      == [[8, 8, 'haberdasher', 'Braden'],[7, 7, 'haberdasher', 'Archit'], ]
test 'g6':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert (Cons.to_vec(result)) \
      == [[3, 4, 'barber', 'Tony']]
test 'g7':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
test 'g8':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(-1, -1, 'barber', 1)
    assert (Cons.to_vec(result))  == [[3.5, 0, 'barber', 'Tony']]
               #    [[0, 3.5, 'barber', 'Judy']]]
test 'g9':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) == [[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
   # [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]

  
test 'g10':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
                 #  [[0, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
 
test 'g11':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
