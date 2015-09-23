#ifndef __CD_H__
#define __CD_H__

#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <functions/RealVector.h>

namespace simulator {

//! @class CD This class define an interface to a generic collision detector that will be used in CR algorithm
class CD {
  public:
    //! @brief This method has to be implemented in every functional CD. 
    //! @brief and the obstacles that have been added. Does not add this trajectory to the system.
    //! @param position Position of all bodies to be checked
    //! @param geometry Geometry of each body
    //! @retval true Some collisions have been detected
    //! @retval false The system is collision-free
    virtual bool detectCollision(const std::vector<std::vector<double> > &position, const std::vector<std::vector<double> > &geometry ) const  = 0;
		
    //! @brief This method has to be implemented in every functional CD. 
    //! @brief and the obstacles that have been added. Does not add this trajectory to the system.
    //! @param position Position of all bodies to be checked
    //! @param geometry One common geometry of the bodies
    //! @retval true Some collisions have been detected
    //! @retval false The system is collision-free
    virtual bool detectCollision(const std::vector<std::vector<double> > &position, const std::vector<double> &geometry ) const  = 0;
    
    
    //! @brief This method has to be implemented in every functional CD. 
    //! @brief and the obstacles that have been added. Does not add this trajectory to the system.
    //! @param position Position of all bodies to be checked
    //! @param geometry Geometry of each body
    //! @retval true Some collisions have been detected
    //! @retval false The system is collision-free
    virtual bool detectCollision1vsAll(const std::vector<std::vector<double> > &position, const std::vector<std::vector<double> > &geometry ) const  = 0;
    
    
    //! @brief This method has to be implemented in every functional CD. 
    //! @brief and the obstacles that have been added. Does not add this trajectory to the system.
    //! @param position Position of all bodies to be checked
    //! @param geometry One common geometry of the bodies
    //! @retval true Some collisions have been detected
    //! @retval false The system is collision-free
    virtual bool segmentCollision(const functions::RealVector &p1_0, const functions::RealVector &p1_1,
				  const functions::RealVector &p2_0, const functions::RealVector &p2_1,
				  const functions::RealVector &geo1, const functions::RealVector &geo2) const = 0; 
  
    //! @brief Loads the information geometry from file into a vector of double
    //! @NOTE The file has to contain float values separated by spaces or end of lines. The derived class will interpret these values.
    //! @param filename Input: the name of the file to load
    //! @param geometry Output: The vector that contains the geometry information
    //! @retval true The file has been loaded successfully
    //! @retval false Problems while reading the file.
    bool loadGeometryFromFile(const std::string &filename, std::vector<double> &geometry) const;
    
    //! @brief gives information about the type of the particular detector
    virtual std::string getType() const = 0;
    
    virtual void expandGeometry(std::vector<double> &geo, double dist) const = 0;
};
}

#endif // __CD_H__