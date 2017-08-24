#include <vector>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

class ImageGrabber
{
public:
	ImageGrabber(){};
	ImageGrabber(string sequencedir, string associationstxt) :massociatxtpath(associationstxt), msequencepath(sequencedir){};
	~ImageGrabber(){};
	void getColorDepthImgs(vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
	{
		LoadImages(massociatxtpath, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
	}
private:
	void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
	{
		ifstream fAssociation;
		fAssociation.open(strAssociationFilename.c_str());
		while (!fAssociation.eof())
		{
			string s;
			getline(fAssociation, s);
			if (!s.empty())
			{
				stringstream ss;
				ss << s;
				double t;
				string sRGB, sD;
				ss >> t;
				vTimestamps.push_back(t);
				ss >> sRGB;
				vstrImageFilenamesRGB.push_back(string(msequencepath + "/" + sRGB));
				ss >> t;
				ss >> sD;
				vstrImageFilenamesD.push_back(string(msequencepath + "/" + sD));
			}
		}
	}
private:
	string massociatxtpath, msequencepath;
};