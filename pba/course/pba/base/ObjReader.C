#include "ObjReader.h"

void ObjReader::read_Obj()
{
	//Open obj file.
	file.open(route.c_str());
	
	while(getline(file, s))
	{
		istringstream iss(s);
		iss >> str1 >> str2 >> str3 >> str4;

		if(s[0] == 'v')
		{
			if(s[1] == 'n')
			{
				vn ++;
				//Store normal
			}
			else
			{
				s2 = std::stof(str2, 0);
				s3 = std::stof(str3, 0);
				s4 = std::stof(str4, 0);
				vnum ++;
				Point p(s2, s3, s4);
				points.push_back(p);
				//Store vertice
			}
		}
		else if(s[0] == 'f')
		{

			if(str2.find("/") != std::string::npos)
			{
				iss >> str5;
				string c1 = str2.substr(0, str2.find("/"));
				string c2 = str3.substr(0, str3.find("/"));
				string c3 = str4.substr(0, str4.find("/"));
				string c4 = str5.substr(0, str5.find("/"));
				float ss1 = stof(c1,0);
				float ss2 = stof(c2,0);
				float ss3 = stof(c3,0);
				float ss4 = stof(c4,0);
				fnum++;
				Face face1(points[ss1 - 1], points[ss2 - 1], points[ss3 - 1]);
				faces.push_back(face1);
				fnum++;
				Face face2(points[ss3 - 1], points[ss4 - 1], points[ss1 - 1]);
				faces.push_back(face2);
			}
			else {
				fnum++;
				s2 = std::stof(str2, 0);
				s3 = std::stof(str3, 0);
				s4 = std::stof(str4, 0);
				Face face(points[s2 - 1], points[s3 - 1], points[s4 - 1]);
				faces.push_back(face);
				//Store face
			}
		}
	}
	cout << "Number of vertices:" << vnum << endl;
	cout << "Number of normals:" << vn << endl;
	cout << "Number of faces:" << fnum << endl;
	
}
