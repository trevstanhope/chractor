#include <iostream>
#include <opencv2/core/core.hpp>


using namespace cv;
using namespace std;

int main() {
	cout << ("OpenCV: %s", cv::getBuildInformation().c_str());
		return 0;
}

