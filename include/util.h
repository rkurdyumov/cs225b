#pragma once
#include <cmath>
#include <math.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <time.h>
#include <boost/math/special_functions/erf.hpp>

using namespace std;

#define PI 3.14159265

class Random
{
  public:
  Random()
  {
    // Reset the random number generator with the system clock
    //srand((unsigned)time(NULL));
  }
  
  // Generate a random number between 0 and 1
  // return a uniform number in [0,1].
  float uniform()
  {
    return rand() / float(RAND_MAX);
  }

  float uniform(float a, float b)
  {
    return (b-a)*uniform() + a;
  }
  
  float gaussian(float mu, float var)
  {
    // Uniformly sampled number [0,1]
    float p = this->uniform();
    // Equivalent to sampling a gaussian with mu,var
    return this->invcdf(mu,var,p);
  }
  
  float invcdf(float mu, float var, float p) {
  {
    if (p < 0.0 || p > 1.0) { p = 0.5;}
      float u = sqrt(2)*boost::math::erf_inv(2*p - 1);
    return sqrt(var)*u + mu;
  }
}
};

class State
{
  public:
  
  State(float newX=0, float newY=0, float newTheta=0)
  {
    this->x = newX;
    this->y = newY;
    this->theta = newTheta;
    this->epsilon = 0.1;
  }
  
  void operator = (const State& otherState)
  {
    this->x = otherState.x;
    this->y = otherState.y;
    this->theta = otherState.theta;
    this->epsilon = otherState.epsilon;
  }
  
  bool operator == (const State& otherState)
  {
    return abs(this->x-otherState.x)<=epsilon && abs(this->y-otherState.y)<=epsilon;
  }
  
  bool operator != (const State& otherState)
  {
    return !(*this == otherState);
  }
  
  bool operator > (const State& otherState)
  {
    return (this->x > otherState.x) && (this->y > otherState.y);
  }
  
  bool operator < (const State& otherState)
  {
    return (this->x < otherState.x) && (this->y < otherState.y);
  }
  
  bool isNull()
  {
    return abs(this->x-0)<=epsilon && abs(this->y-0)<=epsilon;
  }
  float x;
  float y;
  float theta;
  float epsilon;
};

class Node
{
  public:
  Node()
  {
    this->state = State();
    this->g = 0;
    this->h = 0;
    this->f = 0;
  }
  Node(State newState)
  {
    this->state = newState;
    this->g = 0;
    this->h = 0;
    this->f = 0;
  }
  bool operator<(const Node& otherNode)
  {
    bool result = (this->h < otherNode.h);
    return result;
  }
  void operator=(const Node& otherNode)
  {
    this->state = otherNode.state;
    this->parent = otherNode.parent;
    this->g = otherNode.g;
    this->h = otherNode.h;
    this->f = otherNode.f;
  }
  bool isNull()
  {
    return this->state.isNull();
  }
  State state;
  float g;  // Path cost to get to node
  float h;  // Heuristic to goal
  float f;  // f=(g + h), where h is heuristic distance from node to goal
  Node* parent;
};

class Utility
{
public:

static float euclideanDistance(State state1, State state2)
{
  return Utility::euclideanDistance(state1.x,state1.y,state2.x,state2.y);
}

static float euclideanDistance(float x1, float y1, float x2, float y2)
{
  return sqrt(pow((float)(x1-x2),2.0) + pow((float)(y1-y2),2.0));
}

static float manhattanDistance(State state1, State state2)
{
  return Utility::manhattanDistance(state1.x,state1.y,state2.x,state2.y);
}

static float manhattanDistance(float x1, float y1, float x2, float y2)
{
  return abs(x1-x2)+abs(y1-y2);
}

// 1-D Gaussian
static float gaussian(float x, float mu, float sigma)
{
  float c = 1.0/(sqrt(2*PI)*sigma);
  float result = c*exp(-pow(x-mu,2.0)/(2*pow(sigma,2.0)));
  return result;
}

static int ringBinarySearch(vector<float>& ring, float ring_angle)
{
    vector<float>::iterator up;
    up = upper_bound(ring.begin(), ring.end(), ring_angle);
    return int(up - ring.begin());
}

// Bound the input, x, between lowerBound an upperBound
  static float bound(float x,const float lower,const float upper)
  {
    if(x < lower)
      x = lower;
    else if(x > upper)
      x = upper;
    
    return x;
  }
  
  // Wrap a given angle between an upper and lower bound (in radians)
  static float wrap(float angle, float lowerAngle = 0, float upperAngle = 2*PI)
  {
    while(angle < lowerAngle)
      angle += 2*PI;
    
    while(angle > upperAngle)
      angle -= 2*PI;
    
    return angle;
  }
  
  static float avg(float beta, float x, float y)
  {
    return (1-beta)*x + beta*y;
  }
  
  static float avg(float a, float b)
  {
    return (a+b)/2.0;
  }
  
  static float geometricMean(float a, float b)
  {
    return sqrt(a*b);
  }
  
  static float max(float a, float b)
  {
    if(a>b)
      return a;
    else
      return b;
  }
  
  static float min(float a, float b)
  {
    if(a<b)
      return a;
    else
      return b;
  }
};

template <typename T>
struct ValueIndexPair
{
  ValueIndexPair()
  {
    value = static_cast<T>(0);
    x = 0;
    y = 0;
  }
  
  T value;
  int x;
  int y;
};

// Wraps a 2 1-D std::vector's below it.  Allows bilinear interpolation.
template <typename T>
class Matrix
{
   public:
   
   Matrix()
   {
     this->init();
   }
   
   Matrix(int maxX, int maxY)
   {
     this->init();
     if(maxX > 0 && maxY > 0)
     	this->resize(maxX, maxY);
   }
   
   void init()
   {
     this->epsilon = 0.1;
   }
   
   void resize(int maxX, int maxY, T value = static_cast<T>(0))
   {
     this->data.resize(maxX);
     for(int x = 0; x < maxX; x++)
       this->data[x].resize(maxY);
     
     for(int x = 0; x < maxX; x++)
     {
       for(int y = 0; y < maxY; y++)
       {
         this->data[x][y] = value;
       }
     }
   }
   
   int maxX()
   {
     if(this->data.size() <= 0)
       return 0;
       
     return (int)this->data.size();
   }
   
   int maxY()
   {
     if(this->data.size() <= 0 || this->data[0].size() <= 0)
       return 0;
       
     return (int)this->data[0].size();
   }
   
   int size()
   {
     return this->maxX()*this->maxY();
   }
   
   ~Matrix()
   {
   }
      
   // Getter wrapper
   T& operator()(int x, int y)
   {
     return this->at((float)x,(float)y);
   }
   
   T& operator()(float x, float y)
   {
     return this->at(x,y);
   }
   
   T& operator()(double x, double y)
   {
     return this->at((float)x,(float)y);
   }
   
   T& operator()(State state)
   {
     return this->at((float)state.x,(float)state.y);
   }
   
   // Getter
   T& at(float x, float y)
   {
    T interpolatedValue;
     
    x = Utility::bound(x,0,this->maxX()-1);
    y = Utility::bound(y,0,this->maxY()-1);

    int upX = floor(x);
    int downX = ceil(x);
    int leftY = floor(y);
    int rightY = ceil(y);
    //
    T a=data[downX][leftY];
    T b=data[downX][rightY];
    T c=data[upX][leftY];
    T d=data[upX][rightY];
    float alpha=abs(y-floor(y));
    float beta=abs(ceil(x)-x);
    interpolatedValue = Utility::avg(beta,Utility::avg(alpha,a,b),Utility::avg(alpha,c,d));

     T* p_interpolatedValue = new T();
     p_interpolatedValue = &interpolatedValue;
     return *p_interpolatedValue;
   }
   
   // Setter
   void assign(State state, T value)
   {
     this->assign(state.x,state.y,value);
   }
   
   void assign(float x, float y, T value)
   { 
     //float zero_distance = sqrt(2.0); // Where gaussian should be approx. zero
     //float mu = 0; float sigma = sqrt(zero_distance)/2.0;
     
     x = Utility::bound(x,0,this->maxX()-1);
     y = Utility::bound(y,0,this->maxY()-1);
     
     int nearestX = round(x);
     int nearestY = round(y);
     this->data[nearestX][nearestY] = value;
   }
   
   bool equals(int x, int y, T value, float newEpsilon = -1)
   {
     float eps = 0;
     if(newEpsilon > 0)
       eps = newEpsilon;
     else
       eps = this->epsilon;
       
     return (abs(this->at(x,y)-value) < eps);
   }
   
   // Find the indices associated with a specified value
   list<State> find(T value)
   {
     list<State>* indices = new list<State>();
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         if(this->equals(x,y,value))
           indices->push_back(State(x,y));
       }
     }
     return *indices;
   }
   
   // Find the indices associated with a range of values, inclusive
   list<State> find(T lower, T upper)
   {
     list<State>* indices = new list<State>();
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         T value = this->at(x,y);
         if(value >= lower && value <= upper)
           indices->push_back(State(x,y));
       }
     }
     return *indices;
   }
   
   void operator=(Matrix<T>& rhs)
   {
     this->epsilon = rhs.epsilon;
     this->data = rhs.data;
   }
   
   Matrix<T>& operator+(Matrix<T>& rhs)
   {
     Matrix<T>* mat = new Matrix<T>(this->maxX(),this->maxY());
     float value;
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         value = this->at(x,y) + rhs.at(x,y);
         mat->assign(x,y,value);
       }
     }
     return *mat;
   }
   
   float ddx(State state)
   {
     return this->ddx(state.x,state.y);
   }
   
   // Take interpolated ddx at given (x,y)
   float ddx(float x, float y)
   {
     /*float interpolatedValue = 0;
     // d/dx{mat} = cross-correlate with [-1 0 1]
     if(x > 0 && x < (this->maxX()-1))
 	interpolatedValue = this->at(x+1,y)-this->at(x-1,y);
	return interpolatedValue;
     
     int upX = floor(x);
     int downX = ceil(x);
     int leftY = floor(y);
     int rightY = ceil(y);
     //
     T a=data[downX][leftY];
     T b=data[downX][rightY];;
     T c=data[upX][leftY];
     T d=data[upX][rightY];
     //
     float alpha=abs(y-floor(y));
     float gradX = Utility::avg(alpha,b-d,a-c);

     return gradX;*/
     
     int x_r = floor(x);
     int y_r = floor(y);
     if (x_r >= 1 && x_r <= this->maxX() - 1)
       return data[x_r+1][y_r] - data[x_r-1][y_r];
     return 0;
   }
   
   // Take interpolated ddx at every (x,y) in matrix
   Matrix<T>& ddx()
   {
     Matrix<T>* mat = new Matrix<T>(this->maxX(),this->maxY());
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         mat->assign(x,y,this->ddx(x,y));
       }
     }
     return *mat;
   }
   
   float ddy(State state)
   {
     return this->ddy(state.x,state.y);
   }
   
   // Take interpolated ddy at given (x,y)
   float ddy(float x, float y)
   {
     /*float interpolatedValue = 0;
     // d/dy{mat} = cross-correlate with [-1 0 1]'
     if(y > 0 && y < (this->maxY()-1))
       interpolatedValue = this->at(x,y+1)-this->at(x,y-1);
       return interpolatedValue;
     
     int upX = floor(x);
     int downX = ceil(x);
     int leftY = floor(y);
     int rightY = ceil(y);
     //
     T a=data[downX][leftY];
     T b=data[downX][rightY];;
     T c=data[upX][leftY];
     T d=data[upX][rightY];
     //
     float beta=abs(ceil(x)-x);
     float gradY = Utility::avg(beta,d-c,b-a);
       
     return gradY;*/

     int x_r = floor(x);
     int y_r = floor(y);
     if (y_r >= 1 && y_r <= this->maxY() - 1)
       return data[x_r][y_r+1] - data[x_r][y_r-1];
     return 0;
   }
   
   // Take interpolated ddy at every (x,y) in matrix
   Matrix<T>& ddy()
   {
     Matrix<T>* mat = new Matrix<T>(this->maxX(),this->maxY());

     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         mat->assign(x,y,this->ddy(x,y));
       }
     }
     return *mat;
   }
   
   float gradient_magnitude(State state)
   {
     return this->gradient_magnitude(state.x,state.y);
   }
   
   // Take interpolated gradient_magnitude at given (x,y)
   float gradient_magnitude(float x, float y)
   {
     return sqrt(pow(this->ddx(x,y),2.0) + pow(this->ddy(x,y),2.0));
   }
   
   // Take interpolated gradient_magnitude at every (x,y) in matrix
   Matrix<T>& gradient_magnitude()
   {
     Matrix<T>* mat = new Matrix<T>(this->maxX(),this->maxY());
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         mat->assign(x,y,this->gradient_magnitude(x,y));
       }
     }
     return *mat;
   }
   
   float gradient_orientation(State state)
   {
     return this->gradient_orientation(state.x,state.y);
   }
   
   // Take interpolated gradient_orientation at given (x,y), in rads
   float gradient_orientation(float x, float y)
   {
     float value;
     float ddx = this->ddx(x,y);
     float ddy = this->ddy(x,y);
     
     value = atan2(ddy,ddx);
     
     // Undefined
     /*if(ddx==0 && ddy==0)
       value = -100000;
     else if(ddx==0 && ddy>0)
       value = PI/2;
     else if(ddx==0 && ddy<0)
       value = -PI/2;
     else if(ddy==0 && ddx>0)
       value = 0;
     else if(ddy==0 && ddx<0)
       value = PI;*/
     
     return value;
   }
   
   // Take interpolated gradient_orientation at every (x,y) in matrix, in rads
   Matrix<T>& gradient_orientation()
   {
     Matrix<T>* mat = new Matrix<T>(this->maxX(),this->maxY());
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         mat->assign(x,y,this->gradient_orientation(x,y));
       }
     }
     return *mat;
   }
   
   // Find the min with an optional min value to consider
   ValueIndexPair<T>& min(T minCutoff = -100000)
   {
     ValueIndexPair<T>* valueIndexPair = new ValueIndexPair<T>();
     T minVal = static_cast<T>(100000);
     T value;
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         value = this->at(x,y);
         if(value < minVal && value > minCutoff)
         {
           valueIndexPair->x = x;
           valueIndexPair->y = y;
           minVal = value;
         }
       }
     }
     valueIndexPair->value = minVal;
     return *valueIndexPair;
   }
   
   ValueIndexPair<T>& max(T maxCutoff = 100000)
   {
     ValueIndexPair<T>* valueIndexPair = new ValueIndexPair<T>();
     T maxVal = static_cast<T>(-100000);
     T value;
     
     for(int x = 0; x < this->maxX(); x++)
     {
       for(int y = 0; y < this->maxY(); y++)
       {
         value = this->at(x,y);
         if(value > maxVal && value < maxCutoff)
         {
           valueIndexPair->x = x;
           valueIndexPair->y = y;
           maxVal = value;
         }
       }
     }
     valueIndexPair->value = maxVal;
     return *valueIndexPair;
   }
   
   // Create and initialize a matrix with a certain value in each index
   static Matrix<T>& initMatrixWithValue(int maxX, int maxY, T value)
   {
     Matrix<T>* mat = new Matrix<T>(maxX, maxY);
     for(int x = 0; x < maxX; x++)
     {
       for(int y = 0; y < maxY; y++)
       {
         mat->assign(x,y,value);
       }
     }
     return *mat;
   }
   
   static Matrix<T>& empty()
   {
     Matrix<T>* mat = new Matrix();
     return *mat;
   }
   
   bool isEmpty()
   {
     bool result = (this->maxX() == 0) && (this->maxY() == 0);
     return result;
   }
   
   private:
   vector< vector<T> > data;
   float epsilon;
};
