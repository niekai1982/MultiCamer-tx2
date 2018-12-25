#include"trajectory.h"


trk_analysis_result trj_analysis_mix(vector<StateType_mix> pop_trk,   float prob_theta, float proportion, int age_thresh /*= 10*/, int dist_thresh /*= 60*/)
{
	trk_analysis_result result;
	result.cout_add = false;
	float trk_distance;
	int trk_age = pop_trk.size();
	vector<Point> trk_cent;
	int total_head = 0;
	float is_head_pro;
	
	for (int j = 0; j < pop_trk.size(); j++)
	{
	
		
		Point cent = Point((pop_trk[j].box.center.x + pop_trk[j].box.size.x) / 2, (pop_trk[j].box.center.y + pop_trk[j].box.size.y) / 2);
		trk_cent.push_back(cent);
		if (pop_trk[j].pro > prob_theta)
		{
			total_head++;
		}
		//total_pro=+pop_trk[j].pro;
	}
	//mean_prob = total_pro / pop_trk.size();
	is_head_pro = float(total_head) / float(pop_trk.size());
	float deta_y = trk_cent[trk_cent.size() - 1].y - trk_cent[0].y;
	float deta_x = trk_cent[trk_cent.size() - 1].x - trk_cent[0].x;
	

	if (deta_y > 0)
		result.trk_direction = 1;
	else
		result.trk_direction = 0;
	trk_distance = sqrt(pow(deta_x, 2) + pow(deta_y, 2));

	if (trk_age > age_thresh && abs(deta_y) > dist_thresh && is_head_pro >proportion)
	{
		result.cout_add = true;
		return result;
	}

	return result;
}