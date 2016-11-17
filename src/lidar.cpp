#include <iostream>
#include <cstring>
#include <stdlib.h>
#include "mat.h"
#include "lidar.h"
//#include <glm/gtx/string_cast.hpp>

Lidar::Lidar(string filename) {
    cout << "Reading lidar data from " << filename << " ..." << endl;
    cout << " " << endl;
    char* fname = (char*)filename.c_str();

	MATFile *pmat;
	mxArray *pList, *pScan, *pCell;
	float *scanList;

	pmat = matOpen(fname, "r");
    if (pmat == NULL) {
        cout << "Error reading from file - aborting!" << endl;
        throw;
    }

	pList = matGetVariable(pmat, "lidar");
	if (pList == NULL) {
		cout << "File does not contain 'lidar' object - aborting!" << endl;
		throw;
	}

	int len = mxGetNumberOfElements(pList);
	for (int i = 0; i < len; i++) {

		pCell = mxGetCell(pList, i);
		pScan = mxGetField(pCell, 0, "scan");

		if (pScan != NULL) {
			int dim = mxGetNumberOfElements(pScan);
			double classID = mxGetClassID(pScan);
			scanList = (float*) mxGetData(pScan);
			std::vector<float> data;

			for (int j = 0; j < dim; j++) {
				data.push_back((float) scanList[j]);
			}

			scans.push_back(data);
		}

		mxDestroyArray(pScan);
	}
}