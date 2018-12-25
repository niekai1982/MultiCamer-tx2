#include"parameters.h"



ifstream infile("parameters.txt");
int ReadParameters(  parameters &parameter)
{

	if (infile.fail())
	{
		//string ss = "open parameter file error!";
		
		cout << "open parameter file error!" << endl;
		return -1;
	}
	char str[MAX_LINE] = { 0 };

	while (infile.getline(str, sizeof(str)))
	{
		cout << str << endl;
		vector<string> v;
		SplitString(str, v, "=");
		if (v[0] == "txt_key")
		{
			parameter.txt_key = std::stoi(v[1]);
		}
		if (v[0] == "save_key")
		{
			parameter.save_key = std::stoi(v[1]);
		}
		if (v[0] == "depth_RL")
		{
			parameter.depth_RL = std::stoi(v[1]);
		}
		if (v[0] == "depth_RH")
		{
			parameter.depth_RH = std::stoi(v[1]);
		}
		if (v[0] == "depth_value")
		{
			parameter.depth_value = std::stoi(v[1]);
		}
		if (v[0] == "min_distance")
		{
			parameter.min_distance = std::stoi(v[1]);
		}
		if (v[0] == "max_age")
		{
			parameter.max_age = std::stoi(v[1]);
		}
		if (v[0] == "min_hits")
		{
			parameter.min_hits = std::stoi(v[1]);
		}
		if (v[0] == "prob_theta")
		{
			parameter.prob_theta = std::atof(v[1].c_str());
		}
		if (v[0] == "proportion")
		{
			parameter.proportion = std::atof(v[1].c_str());
		}
		if (v[0] == "age_thresh")
		{
			parameter.age_thresh = std::atof(v[1].c_str());
		}
		if (v[0] == "dist_thresh")
		{
			parameter.dist_thresh = std::atof(v[1].c_str());
		}
	}
	infile.close();
	return 0;




}

void SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}



