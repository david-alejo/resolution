#ifndef __POINT_3D_H__
#define __POINT_3D_H__

#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <vector>

#include "functions.h"

/** @brief 3D Point structure and operators

 */

namespace functions {

template<typename T> T sign(const T&num);
double reduceAngle(double angle);

class Point3D{
		
		
	public:
		
	//! Internal data
		double x;
		double y;
		double z;
			
	
	protected:
		
		//! Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//! init without arguments -> all internal data to empty or zero values
		void init();
		
	public:
		Point3D();
	
		Point3D(const double &px, const double &py, const double &pz);
	
		Point3D (const Point3D &that);
		
		//! @brief Constructor from a std vector.
		//! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
		Point3D(const std::vector<double> &vec);
		
		//! @brief Initializing method from a standard vector
		//! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
		void init(const std::vector<double> &vec);
		
		//!Destructor
		~Point3D();
		
		//! Prefer clone to copy constructor.
		Point3D * clone () const;  
			
		//!Returns a string representation of the object
		std::string toString(bool format = true) const;
		
		//!addition operator (sums each coordinate)
		friend const Point3D operator+(const Point3D &left, const Point3D &right);
		
		//!substraction operator(substracts each coordinate)
		friend const Point3D operator-(const Point3D &left, const Point3D &right);
		
		//!multiply operator(multiplies each coordinate by a constant)
		friend const Point3D operator*(const Point3D &left, const double &right);
		
		//!dot product operator
		friend  double operator*(const Point3D &left, const Point3D &right);
		
		//!exponentiation operator( exponentiates each coordinate to a given power)
		friend const Point3D operator^(const Point3D &left, const double &right);
		
		//!scalar division  operator
		friend const Point3D operator/(const Point3D &left, const double &right);
		
		//!operator opposite (changes sign to each coordinate)
		friend const Point3D operator-(const Point3D &a);
		
		//! @brief Addition assignment operator
		Point3D &operator += (const functions::Point3D& that);
		
		//! @brief Substraction assignment operator
		Point3D &operator -= (const functions::Point3D& that);
		
		//! assignment operator
		Point3D & operator = (const Point3D &that);
		
		inline double distance(const Point3D &p)const {
		  Point3D d(p - *this);
		  return d.norm();
		}
		  
		inline double distance2d(const Point3D &p) const {
		  Point3D d(p - *this);
		  return d.norm2d();
		}
		
		inline double sqr() const {
		  return  x*x + y*y + z*z;
		}
		
		inline double norm()const {
		  return sqrt(sqr());
		}
		
		inline double norm2d() const {
		  return sqrt(sqr2d());
		}
		
		inline double sqr2d() const {
		  return  x*x + y*y;
		}
		
		Point3D crossProduct(const Point3D &p)const;
		
		//! @brief normalizes the vector
		inline void normalize() {
			double norm_ = norm();
			
			if (norm_ > FUNC_EPSILON) {
			
			  x /= norm_;
			  y /= norm_;
			  z /= norm_;
			}
		}
		
		//! @return The increment in the course angle (2D)
		inline double getDeltaHeading(const Point3D &p) {
		  return reduceAngle(p.getHeading() - this->getHeading());
		}
		
		//! @return The increment in the course angle (2D)
		inline double getHeadingTo(const Point3D &p) const {
		  return (p - *this).getHeading();
		}
		
		inline double getHeading() const {
		  if (fabs(x) < FUNC_EPSILON) {
		    return (sign(y) < 0.0)?-M_PI*0.5:M_PI*0.5;
		  }
		  return atan2(y, x);
		}
		
		//! @brief Returns the angle between two vectors (cosine theorem)
		inline double getAngle(const Point3D &p) const {
		  if (this->norm() * p.norm() < FUNC_EPSILON) {
		    return 0.0;
		  }
		  return acos( (*this * p) / (this->norm() * p.norm()) );
		}
		
		//! @brief Returns the angle between two 2-dimensional vectors (cosine theorem)
		inline double getAngle2d(const Point3D &p) const {
		  if (sqr2d() < FUNC_EPSILON) {
		    return 0.0;
		  }
		  return acos( ((p.x * this->x) + (p.y * this->y)) / (sqr2d()) );
		}
		
		

		/** Initializing method
		 * @param d internal data
		*/
		void init(const double &px, const double &py, const double &pz);	
};

}

#endif //__POINT_3D_H__
