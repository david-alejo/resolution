#include <RealVector.h>
#include <functions.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace functions;
using namespace std;

int main(int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage " << argv[0] << " <input_file> <output_file> <vector>\n";
    exit(-1);
  }
  
  vector<vector<double> > v;
  
  if(getMatrixFromFile(string(argv[1]), v) && v.size() > 0) {
    RealVector disp((int)v.at(0).size());
    
    try {
      for (int i = 3; i < argc; i++) {
	disp.at(i - 3) = boost::lexical_cast<double>(argv[i]);
      }
    } catch (boost::bad_lexical_cast const&) {
      cerr << "Could not retrieve the displacement vector\n";
      exit (-3);
    }
    
    cout << "Tranlator: " << disp.toString() << endl;
    
    ofstream ofs(argv[2]);
    
    if (ofs.is_open()) {
      for (unsigned int i = 0; i < v.size(); i++) {
	RealVector v_curr(v.at(i));
	v_curr = v_curr + disp;
	ofs << v_curr.toMatlabString() << endl;
      }
      ofs.close();
    } else {
      cerr << "Could not open the output file.\n";
      exit (-4);
    }
    
  } else {
    cerr << "Could not open the file.\n";
    exit (-2);
  }
  
  return 0;
  
}