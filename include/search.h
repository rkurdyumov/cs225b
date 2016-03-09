#pragma once
#include "util.h"
#include <queue>
#include <list>
#include <set>
#include <map>
using namespace std;

#define INITIALIZER_LARGE 100000

float nullHeuristic(State state)
{
  return 0;
}

class wavefront_comparison
{
  public:
  bool operator() (const Node* lhs, const Node* rhs) const
  {
    bool result = (lhs->h > rhs->h);
    return result;
  }
};
  
class a_star_comparison
{
  public:
  bool operator() (const Node* lhs, const Node* rhs) const
  {
    bool result = (lhs->f > rhs->f);
    return result;
  }
};

class ucs_comparison
{
  public:
  bool operator() (const Node* lhs, const Node* rhs) const
  {
    bool result = (lhs->g > rhs->g);
    return result;
  }
};

class bfs_comparison
{
  public:
  bool operator() (const Node* lhs, const Node* rhs) const
  {
    bool result = false;
    return result;
  }
};

#define X_MAP 9999.0
#define Y_MAP 2.0
class node_comparison
{
  public:
  bool operator() (const Node* lhs, const Node* rhs) const
  {
    State lhsState = lhs->state;
    State rhsState = rhs->state;
    return (round(X_MAP*lhsState.x+Y_MAP*lhsState.y)) < (round(X_MAP*rhsState.x+Y_MAP*rhsState.y));
  }
};

class Search
{
  public:

  // Potential wave propagation function
  Node wavefrontPotential(State startState,boost::function<list<State>& (State&)>& getSuccessors,boost::function<bool (Node&)>& isGoal,Matrix<float>& I,Matrix<float>& h)
  {    
    map<int,Node*> visitedNodes;
    priority_queue<Node*,vector<Node*>,wavefront_comparison> frontierNodes;
    
    Node* rootNode = new Node(startState);
    rootNode->parent = NULL;
    frontierNodes.push(rootNode);
    
    // Set the potential at the start state to be zero
    h.assign(startState.x,startState.y,0);
  
    int numIters = 0;
    while(!frontierNodes.empty())// && numIters < 50)
    {
      numIters++;
      // Grab the next node in line to be expanded
      Node* currNode = frontierNodes.top();
      frontierNodes.pop();
       
      // Case we've already visited this state, so skip over it
      int key = round(currNode->state.x*X_MAP+currNode->state.y*Y_MAP);
      map<int,Node*>::iterator preexistingIter = visitedNodes.find(key);
      if(preexistingIter != visitedNodes.end())
        continue;

      // Case this is our goal state, so end the search
      if( isGoal(*currNode) )
      {
          return *currNode;
      }
      
      // Make sure we don't visit this state again
      visitedNodes[key] = currNode;
        
      // Find the children of this state
      list<State> successorStates = getSuccessors(currNode->state);
    
      list<State>::iterator iter;
      for(iter = successorStates.begin(); iter != successorStates.end(); iter++)
      {
        State successorState = *iter;
        Node* newNode = new Node(successorState);
        newNode->parent = currNode;
        
        // Find this child's 4 nearest neighbors and sort them according to
        //     their h values
        list<State> childsSuccessorStates = getSuccessors(newNode->state);
        list<Node> childsSuccessorNodes;
        for(list<State>::iterator iter = childsSuccessorStates.begin(); iter != childsSuccessorStates.end(); iter++)
        {
          Node nextChild = Node(*iter);
          nextChild.h = h(*iter);
          childsSuccessorNodes.push_back(nextChild);
        }
        if(childsSuccessorNodes.size() > 0)
	  {
	    childsSuccessorNodes.sort();
	  }
        
        // The cost of moving from (a->c) or (b->c) is called F and is 1 plus the
        //     intrinsic cost at c
        float F = I(newNode->state)+1;
        bool noChildren = false;
        
        Node aNode;
        float a = 0;
        if(childsSuccessorNodes.size() > 0)
	  {
	    aNode = childsSuccessorNodes.front();
	    a = aNode.h;
	    childsSuccessorNodes.pop_front();
	  }
        else
	  {
	    // Since there are no children, set c equal to a large number, so that
	    //     this newNode will be ignored
	    noChildren = true;
	  }
	
	Node bNode;
	float b = 0;
	if (childsSuccessorNodes.size() > 0) {
	  bNode = childsSuccessorNodes.front();
	  b = bNode.h;
	  childsSuccessorNodes.pop_front();
	}
	else
	  {
	    // Default to the case where b>>a so c=a+F
	    b = 2*(a+F);
	    bNode = aNode;
	    bNode.state.x = aNode.state.y = -1;
	  }
        
        // Case the 2nd and 3rd smallest values are equal so choose b as the
        //     closest via euclidean distance to a
        if(childsSuccessorNodes.size() > 0)
	  {
	    Node c_primeNode = childsSuccessorNodes.front();
	    float c_prime = bNode.h;
	    childsSuccessorNodes.pop_front();
          
	    if(c_prime == b && (Utility::euclideanDistance(aNode.state,bNode.state) > Utility::euclideanDistance(aNode.state,c_primeNode.state)))
	      {
		b = c_prime;
		bNode = c_primeNode;
	      }
	  }
        
        float c = INITIALIZER_LARGE;
        if(!noChildren)
	  {
	    if((aNode.state.y == bNode.state.y) || (aNode.state.x == bNode.state.x))
	      {
		// a and b are on opposite sides of c
		c = (a+b)/2.0;
	      }
	    else if(a==b)
	      c = a+(1.0/sqrt(2.0))*F;
	    else if(b > a+F)
	      c = a+F;
	    else
	      {
		// (a<=b && b<=a+F)
		c = (a+b)/2.0+sqrt(pow(a+b,2.0)-2*(pow(a,2.0)+pow(b,2.0)-pow(F,2.0)))/2.0;
	      }
	  }
        
        float newH = c;
        
        // If the newH is smaller than the oldH, replace the h values and add 
        //     this node to the frontierNodes queue
        if(newH < h(newNode->state))
	  {
	    h.assign(newNode->state,newH);
	    newNode->h = newH;
	    frontierNodes.push(newNode);
	  }
      }
    }
    
    return Node();
  }
  
  // Main search functions
  Node a_star(list<State> startStates,boost::function<list<State>& (State&)>& getSuccessors,boost::function<bool (Node&)>& isGoal,float (*h)(State state), Matrix<float>& storage = Matrix<float>::empty(),bool allowRevisits = true)
  {
    return this->search<a_star_comparison>(startStates,getSuccessors,isGoal,h,storage,allowRevisits);
  }
  
  Node ucs(list<State> startStates,boost::function<list<State>& (State&)>& getSuccessors,boost::function<bool (Node&)>& isGoal,Matrix<float>& storage = Matrix<float>::empty(),bool allowRevisits = true)
  {
    return this->search<ucs_comparison>(startStates,getSuccessors,isGoal,&nullHeuristic,storage,allowRevisits);
  }

  Node bfs(list<State> startStates,boost::function<list<State>& (State&)>& getSuccessors,boost::function<bool (Node&)>& isGoal,Matrix<float>& storage = Matrix<float>::empty(),bool allowRevisits = true)
  {
    return this->search<bfs_comparison>(startStates,getSuccessors,isGoal,&nullHeuristic,storage,allowRevisits);
  }

  private:
  template <class NodeComparisonClass>
  Node search(list<State> startStates,boost::function<list<State>& (State&)>& getSuccessors,boost::function<bool (Node&)>& isGoal,float (*h)(State state),Matrix<float>& storage = Matrix<float>::empty(),bool allowRevisits = true)
  {    
    map<int,Node*> visitedNodes;
    priority_queue<Node*,vector<Node*>,NodeComparisonClass> frontierNodes;    
       
    for(list<State>::iterator iter = startStates.begin(); iter != startStates.end(); iter++)
    {
      Node* rootNode = new Node(*iter);
      rootNode->parent = NULL;
      frontierNodes.push(rootNode);
    }
  
    Node* goalNode = NULL;
    while(!frontierNodes.empty())
    {
      // Grab the next node in line to be expanded
      Node* currNode = frontierNodes.top();
      frontierNodes.pop();
            
      // Case we've already visited this state, so skip over it
      int key = round(currNode->state.x*X_MAP+currNode->state.y*Y_MAP);
      map<int,Node*>::iterator preexistingIter = visitedNodes.find(key);
      if(preexistingIter != visitedNodes.end())
      {
        if(!allowRevisits)
        {
          continue;
        }
        else
        {  
          Node* preexistingNode = preexistingIter->second;
          if(currNode->g >= preexistingNode->g)
            continue;
        }
      }
     
      // Make sure we don't visit this state again
      visitedNodes[key] = currNode;

      // Case this is our goal state, so end the search
      if( isGoal(*currNode) )
      {
          goalNode = currNode;
          continue;
      }
        
      // Find the children of this state and add them to the frontier
      list<State> successorStates = getSuccessors(currNode->state);
    
      list<State>::iterator iter;
      for(iter = successorStates.begin(); iter != successorStates.end(); iter++)
      {
        State successorState = *iter;
        Node* newNode = new Node(successorState);
        newNode->parent = currNode;

        // This node's cost, g, is the sum of its parent's cost + its distance
        //     from its parent
        newNode->g = currNode->g + Utility::euclideanDistance(currNode->state,newNode->state);
        newNode->f = newNode->g + (*h)(newNode->state);

        // If we want to store the distance values in memory for later retrieval
        if(!storage.isEmpty())
        {
          if(newNode->g < storage(newNode->state.x,newNode->state.y))
          {
            storage.assign(newNode->state.x,newNode->state.y,newNode->g);
          }
        }
          
        frontierNodes.push(newNode);
      }
    }
    
    Node resultNode;
    if(goalNode != NULL)
      resultNode = *goalNode;
    else
      resultNode = Node();
    return resultNode;
  }
};

