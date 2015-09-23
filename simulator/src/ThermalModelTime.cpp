/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "ThermalModelTime.h"
#include <functions/functions.h>

using namespace std;

namespace simulator {

ThermalModel* ThermalModelTime::clone() const
{
    ThermalModel * ret = new ThermalModelTime();
    ThermalModelTime *other = dynamic_cast<ThermalModelTime *>(ret);
    other->filename = filename;
    other->matrix = matrix;
    other->creation_time = creation_time;
    other->t = t;
    other->data = data;
    other->resolution = resolution;
    other->filename = filename;
    other->directory = directory;
    other->max_t = max_t;
    
    return ret;
}

ThermalModel* ThermalModelTime::createModel(ParseBlock& b) const
{
  Checker *check = getChecker();
  ThermalModel *ret = NULL;

  try {
    b.checkUsing(check);
    
    ret = new ThermalModelTime;

    ThermalModelTime *tmf = static_cast<ThermalModelTime *>(ret);
		  
    tmf->resolution = b("resolution").as<double>();
    tmf->filename = b("wind_file").value;
    tmf->directory = b("directory").value;
    tmf->max_t = b("max_t").as<int>();
    tmf->t = b("t").as<int>();
    
    // Load all the files
    for (int i = 1; i <= tmf->max_t; i += tmf->t) {
      ostringstream os;
      os << tmf->directory << "/" << tmf->filename << i;
      cout << "ThermalModelTime::createModel --> Loading file: " << os.str() << endl;
      tmf->loadData(os.str());
      tmf->matrix.push_back(tmf->data);
    }
    tmf->creation_time.getTime(); // Register the creation time
  } catch (exception &e) {
    std::cerr << "ThermalModelTime::createModel --> Error: could not load the data." << e.what() << endl;
    throw(e);
  }
  delete check;
  
  return ret;
}

Checker* ThermalModelTime::getChecker() const
{
    Checker *check = simulator::ThermalModelFile::getChecker();
    
    check->addProperty("t", new NTimes(1));
    check->addProperty("max_t", new NTimes(1));
    check->addProperty("directory", new NTimes(1));
    
    return check;
}

double ThermalModelTime::getVerticalWindSpeed(const functions::RealVector& v) const
{
  functions::FormattedTime curr_t;
  curr_t.getTime();
  
  
  return getVerticalWindSpeed(v, curr_t - creation_time);
}

double ThermalModelTime::getVerticalWindSpeed(const functions::RealVector& v, double sim_t) const {
  int index = sim_t / t;
  
  // Saturate the index between the desired bounds
  index = functions::saturate(index, 0, static_cast<int>(matrix.size() - 1));
  
  double ret = simulator::ThermalModelFile::getVerticalWindSpeed(v, matrix.at(index));
  
  if (ret > 0.0) {
    ret *= 2.0;
  }
  
  return ret;
}

ThermalModelTime::~ThermalModelTime()
{
  dispose();
}


  void ThermalModelTime::dispose()
  {
    data.clear();
    matrix.clear();
  }

  
ParseBlock* ThermalModelTime::exportThermalData() const
{
  ParseBlock *block = new ParseBlock;
  
  string res = functions::numberToString(resolution);
  block->setProperty("resolution", res);
  block->setProperty("wind_file", filename);
  block->setProperty("type", "Time");
  block->setProperty("t", functions::numberToString(t));
  block->setProperty("max_t", functions::numberToString(max_t));
  block->setProperty("directory", directory);
  
  return block; 
}

}
