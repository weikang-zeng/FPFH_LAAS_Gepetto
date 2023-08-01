#include "gmm.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace std;





/*const vector<double> &inputData,*/
void GMM::Init(const int clustNum, double eps, double max_steps)
{
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr inputData(new pcl::PointCloud<pcl::FPFHSignature33>);
	//pcl::io::loadPCDFile<pcl::FPFHSignature33> ("/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features/features_s0.010469_plateforms_r1.340000_px0.012208_py0.212959_pz0.924083_ox-0.431771_oy-0.140291_oz0.847398_ow-0.275336.pcd",*inputData);
	/*get input data*/
	this->data = inputData;
	this->dataNum = data.size();
	/*Store the final desired result*/
	this->clusterNum = clustNum;		// Number of clusters
	this->epslon = eps;					// threshold
	this->max_steps = max_steps;		// maximum number of iterations
	/*Keep the mean and variance parameters of each category, keep the previous parameter*/
	this->means.resize(clusterNum);
	this->means_bkp.resize(clusterNum);
	this->sigmas.resize(clusterNum);
	this->sigmas_bkp.resize(clusterNum);
	/*The probability of retaining each data for each category is obtained by dividing the probability under this category by the total probability of each category*/
	this->memberships.resize(clusterNum);
	this->memberships_bkp.resize(clusterNum);
	for (int i = 0; i < clusterNum; i++)
	{
		memberships[i].resize(data.size());
		memberships_bkp[i].resize(data.size());
	}
	/*Possibilities for each category*/
	this->probilities.resize(clusterNum);
	this->probilities_bkp.resize(clusterNum);
	//initialize mixture probabilities 初始化每个类别的参数
	for (int i = 0; i < clusterNum; i++)
	{
		probilities[i] = probilities_bkp[i] = 1.0 / (double)clusterNum;
		//init means
		means[i] = means_bkp[i] = 255.0*i / (clusterNum);
		//init sigma
		sigmas[i] = sigmas_bkp[i] = 50;
	}
}

void GMM::train()
{
	//compute membership probabilities
	int i, j, k, m;
	double sum = 0, sum2;
	int steps = 0;
	bool go_on;
	do			// 迭代
	{
		for (k = 0; k < clusterNum; k++)
		{
			//compute membership probabilities
			for (j = 0; j < data.size(); j++)
			{
				//Calculate p(k|n), calculate the weighted sum of the values of each data in each category
				sum = 0;
				for (m = 0; m < clusterNum; m++)
				{
					sum += probilities[m] * gauss(data[j], means[m], sigmas[m]);
				}
				//Find the numerator, the proportion of the jth data in the kth class
				memberships[k][j] = probilities[k] * gauss(data[j], means[k], sigmas[k]) / sum;
			}
			//find the mean
			//Find the sum of the conditional probabilities, and accumulate the probability of each data in the kth class
			sum = 0;
			for (i = 0; i < dataNum; i++)
			{
				sum += memberships[k][i];
			}
			//Get the sum of the weighted values of each data belonging to the kth class
			sum2 = 0;
			for (j = 0; j < dataNum; j++)
			{
				sum2 += memberships[k][j] * data[j];
			}
			//Get the new mean by dividing the weighted sum of the probabilities by the total probability as the mean
			means[k] = sum2 / sum;
			//Find the variance from the weighted sum of squares to the mean as the new variance
			sum2 = 0;
			for (j = 0; j < dataNum; j++)
			{
				sum2 += memberships[k][j] * (data[j] - means[k])*(data[j] - means[k]);
			}
			sigmas[k] = sqrt(sum2 / sum);
			//Find the probability
			probilities[k] = sum / dataNum;
		}//end for k
		//check improvement
		go_on = false;
		for (k = 0; k<clusterNum; k++)
		{
			if (means[k] - means_bkp[k]>epslon)
			{
				go_on = true;
				break;
			}
		}
		//back up
		this->means_bkp = means;
		this->sigmas_bkp = sigmas;
		this->probilities_bkp = probilities;
	} while (go_on&&steps++ < max_steps); //end do while
}


//this part represente probability density function
double GMM::gauss(const double x, const double m, const double sigma)
{
	return 1.0 / (sqrt(2 * 3.1415926)*sigma)*exp(-0.5*(x - m)*(x - m) / (sigma*sigma));   //sigma=covariance
}


int GMM::predicate(double x)
{
	double max_p = -100;
	int i;
	double current_p;
	int bestIdx = 0;
	for (i = 0; i < clusterNum; i++)
	{
		current_p = gauss(x, means[i], sigmas[i]);
		if (current_p > max_p)
		{
			max_p = current_p;
			bestIdx = i;
		}
	}
	return bestIdx;
}
void GMM::print()
{
	int i;
	for (i = 0; i < clusterNum; i++)
	{
		cout << "Mean: " << means[i] << " Sigma: " << sigmas[i] << " Mixture Probability: " << probilities[i] << endl;
	}
}