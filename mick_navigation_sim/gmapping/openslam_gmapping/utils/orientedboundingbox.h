#ifndef  ORIENTENDBOUNDINGBOX_H
#define  ORIENTENDBOUNDINGBOX_H


#include <stdio.h>
#include <math.h>

#include <utils/point.h>

namespace GMapping{

template<class NUMERIC>
class OrientedBoundingBox {

 public:
  OrientedBoundingBox(std::vector< point<NUMERIC> > p);  
  double area();

 protected:
  Point ul;
  Point ur;
  Point ll;
  Point lr;  
};

#include "orientedboundingbox.hxx"

};// end namespace

#endif

