#include <math.h>
#include <stdio.h>

class Vector2d{
 public: 
  Vector2d(){
    x = 0.;
    y = 0.;
    
  }
  Vector2d(double bearing){
    x = cos(bearing);
    y = sin(bearing);
   
  }
  Vector2d(double aX, double aY){
    x = aX;
    y = aY;
   
  }
  ~Vector2d(){};

  // Get Methods
  double GetX(){return x;}
  double GetY(){return y;}
  double GetNorm(){return sqrt(x*x + y*y);}
  double GetBearing(){return atan2(y,x);}

  // Vector Operations
  double DotProduct(Vector2d* aVec){
    return x*aVec->GetX() + y*aVec->GetY();
  }
  
  Vector2d* MultiplyByScalar(double s){
    double xNew = s*x;
    double yNew = s*y;  

    return new Vector2d(xNew, yNew); 
  }

  Vector2d* GetPerpVector(){
    double bearing = atan2(x,-y);

    return new Vector2d(bearing);
  }

  // This function is used mainly for debuggin
  void Print(){
    printf("%.2f %.2f %.2f %.2f \n", x, y, GetNorm(), GetBearing());
  }

 private:
  double x;
  double y;
};
