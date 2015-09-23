#include "ThermalModelFile.h"
#include "functions/functions.h"

using namespace std;

namespace simulator {

	ThermalModel *ThermalModelFile::createModel(ParseBlock &b) const {
		Checker *check = getChecker();
		ThermalModel *ret = NULL;

		try {
		  b.checkUsing(check);
		  
		  ret = new ThermalModelFile;

		  ThermalModelFile *tmf = static_cast<ThermalModelFile *>(ret);
		  
		  tmf->resolution = b("resolution").as<double>();
		  tmf->filename = b("wind_file").as<string>();

		  cout << "ThermalModel::createModel --> Loading file: " << tmf->filename << endl;

		  tmf->loadData(tmf->filename);

//		  cout << "Data loaded. Content: " << functions::printMatrix(tmf->data);
		} catch (exception &e) {
		  std::cerr << "ThermalModelFile::createModel --> Error: could not load the data." << e.what() << endl;
		  throw(e);
		}
		return ret;
	}

	bool ThermalModelFile::loadData(const std::string &filename) {
		return functions::getMatrixFromFile(filename, data);
	}

	Checker *ThermalModelFile::getChecker() const {
		Checker *check = ThermalModel::getChecker();

		check->addProperty("resolution", new NTimes(1));
		check->addProperty("wind_file", new NTimes(1));

		return check;
	}

	// TODO: complete this
	ParseBlock *ThermalModelFile::exportThermalData() const {
		ParseBlock *block = new ParseBlock;

		string res = functions::numberToString(resolution);
		block->setProperty("resolution", res);
		block->setProperty("wind_file", filename);
		block->setProperty("type", "File");

		return block;
	}

double ThermalModelFile::getVerticalWindSpeed(const functions::RealVector& v, const WindMatrix& data) const
{
  if (v.size() < 2 || data.size() == 0 || data.at(0).size() == 0 ) {
    // No valid input vector data
    return 0.0;
  }

  if (v.at(0) < 0.0 || v.at(0) > data.size() * resolution) {
    return 0.0; // out of the scope
  }

// cout << "ThermalModelFile::getVerticalWindSpeed() --> v = " << functions::printVector(v) << endl;
// cout << floor(v.at(0) / data.size() * resolution) << "\t";

  int first = (int)floor(v.at(0) / resolution);
  int second = (int)floor(v.at(1) / resolution);

// cout << "ThermalModelFile::getVerticalWindSpeed() --> first = " << first;
// cout << "\tsecond = " << second << endl;

  if (first < 0 || first > data.size() || second < 0 || second > data.at(first).size()) {
    return 0.0; // out of the scope
  }

//cout << "return = " << data[first][second] << endl;

    return data[first][second];
  }

ThermalModel *ThermalModelFile::clone() const {
  ThermalModel *r = new ThermalModelFile;
  ThermalModelFile *tmf = dynamic_cast<ThermalModelFile *>(r);
  tmf->data = data;
  tmf->resolution = resolution;
  tmf->filename = filename;

  return r;
}
	
  double ThermalModelFile::getVerticalWindSpeed(const functions::RealVector &v) const {
    return getVerticalWindSpeed(v, data);
  }

  
ThermalModelFile::~ThermalModelFile()
{
  data.clear();
  filename.clear();
}

}

