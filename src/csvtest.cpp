#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>

using namespace std;


int main(){

  string drone_info_path = "../data/data_preprocessed.csv";
  //open up the file for reading

 
  string line;
  
  while ( file.good() ) {
    
    getline ( file, line, '\n' ); 

    vector<string> strs;
    boost::split(strs, line, boost::is_any_of(","));

    cout << line << '\n'; 

    for (int i = 0; i < strs.size(); i++) {
      
      cout << i << ":" << strs[i] << " ";

    }

    cout << '\n'; 

  }

}
