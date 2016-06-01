#include <stdio.h>
#include "Vector2d.h"

int main(){

  Vector2d * a = new Vector2d(0.76);
  
  Vector2d * aPerp = a->GetPerpVector();
  Vector2d * aNeg = a->MultiplyByScalar(-1.);
  Vector2d * aNegPerp = aNeg->GetPerpVector();
  Vector2d * aScaled = a->MultiplyByScalar(3.);

  printf("a:\t");
  a->Print();
  
  printf("aPerp:\t");
  aPerp->Print();

  printf("aNeg:\t");
  aNeg->Print();

  printf("aNegPerp:\t");
  aNegPerp->Print();

  printf("aScaled:\t");
  aScaled->Print();

  printf("a dot aNeg:\t%f\n", aNeg->DotProduct(a));
  printf("a dot aPerp:\t%f\n", aPerp->DotProduct(a));  
  printf("a dot aNegPerp:\t%f\n", aNegPerp->DotProduct(a));  

  return 0;
}
