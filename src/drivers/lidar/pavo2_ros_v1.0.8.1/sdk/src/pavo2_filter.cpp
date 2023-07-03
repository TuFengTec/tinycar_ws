#include <string.h>
#include <limits>
#include "angles.h"
#include <set>
#include "pavo2_filter.h"
#include <cmath>
#include <iostream>
#define PI 3.141592653
#define Degree2Radians(X) ((X)/180*PI)
using namespace std;

int g_iIntensity_table[3] = { 10,13,15 };

MADataFilter::MADataFilter()
{
	_distance_diff = 50;
	_intensity_diff = 10;
	_window_size = 5;
	tailfilter_method = 6;
	tailfilter_method_angle = 0;
	_window_vector.resize(_window_size);
	front_filter_level = 0;
}

pavo_response_scan_t* MADataFilter::Filter(pavo_response_scan_t* data_buffer,int count)
{
	if (count <= _window_size) return data_buffer;

	int method = 0;

	if (tailfilter_method)
	{
		method = tailfilter_method;
		if (method == 1)
		{
			HandlTialFilter(data_buffer, count);
		}
		if (method == 2)
		{
			HandlTialFilterWithRatio(data_buffer, count);
		}
		if (method == 3)
		{
			HandlTialFilterWithDistance_1(data_buffer, count);
		}
		if (method == 4)
		{
			HandlTialFilterWithDistance_2(data_buffer, count);
		}
		if (method == 5)
		{
			HandlTialFilterWithDistanceForContinuity(data_buffer, count);
		}
		if (method == 6)
			HandlTialFilterWithAngle(data_buffer, count);
	}
	HandlFrontFilter(data_buffer,count);
	return data_buffer;
}

pavo_response_scan_t*  MADataFilter::HandlTialFilterWithDistanceForContinuity(pavo_response_scan_t* data_buffer, int count)
{
	const  double dis_diff = 10;
	// the tailer range start and end
	int location_start;
	int location_end;
	// the lianxu num
	int  continuity_num;
	// the proceess state 
	int state; //1: tailer state  ;0  continue state
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];
	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	location_start = 0;
	location_end = 0;
	continuity_num = 0;
	state = 0;
	for (int i = 0; i < count; i++)
	{
		if (i == 0)
		{
			continue;
		}

		double dis1 = copy_ptr_org[i].distance;
		double dis2 = copy_ptr_org[i - 1].distance;
		double k = (dis2 - dis1);
		if (k > dis_diff || k < -dis_diff)
		{
			if (state == 0)
			{
				state = 1;
				location_start = i;
			}
			continuity_num = 0;
			continue;
		}
		continuity_num++;
		if (continuity_num >= 2 && state == 1)
		{
			state = 0;
			location_end = i - continuity_num;
			for (int j = location_start; j < location_end; j++)
			{
				copy_ptr[j].distance = 0;
			}
		}
	}
	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;

}

inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}

pavo_response_scan_t* MADataFilter::HandlTialFilterWithAngle(pavo_response_scan_t* data_buffer, int count)
{
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];
	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	double min_angle_ = 2.5;
	double max_angle_ = 177.5;
	int window_ = 5;
	int neighbors_ = 2;
	int remove_method = 1;
	float distance_def, distance_std = 0;
	if (tailfilter_method_angle)
	{
		remove_method = tailfilter_method_angle;
	}
	else {
		remove_method = 0;
	}

	switch (remove_method)
	{
	case 0:
		neighbors_ = 0;
		distance_def = 0;
		break;
	case 1:
		neighbors_ = 1;
		min_angle_ = 5;
		max_angle_ = 175;
		distance_def = 10;
		break;
	case 2:
		neighbors_ = 2;
		min_angle_ = 8;
		max_angle_ = 172;
		distance_def = 6;
		break;
	case 3:
		neighbors_ = 3;
		min_angle_ = 10;
		max_angle_ = 170;
		distance_def = 3;
		break;
	default:
		neighbors_ = 2;
		distance_def = 0;
		break;
	}
	distance_std = distance_def*1.0 / 0.2;
	unsigned short angle1, angle2;

	std::set<int> indices_to_delete;
	float dandiao1, dandiao2;
	indices_to_delete.clear();
	// For each point in the current line scan
	for (unsigned int i = 0; i < count; i++)
	{
		if (copy_ptr_org[i].distance == 0)
		{
			continue;
		}

		if (false && (indices_to_delete.find(i) != indices_to_delete.end()))
		{
			continue;
		}

		for (int y = -window_; y < window_ + 1; y++)
		{
			int j = i + y;
			if (j < 0 || j >= count || (int)i == j)
			{
				// Out of scan bounds or itself
				continue;
			}
			dandiao1 = 1; 
			dandiao2 = 1;
			double angle1, angle2, angle;
			angle1 = getAngleWithViewpoint(float(copy_ptr_org[i].distance), float(copy_ptr_org[j].distance), y * Degree2Radians(float((copy_ptr_org[i + 1].angle - copy_ptr_org[i].angle) / 100.0f)));
			angle2 = angles::to_degrees(angle1);
			angle = std::abs(angle2);
			if (j < i)
			{
				if (j == (i - 1))
				{
					dandiao1 = dandiao1*angle1;
				}

			}
			else
			{
				if (j == (i + 1))
				{
					dandiao2 = dandiao2*angle1;
				}
			}
			//std::cout<<copy_ptr_org[i+1].angle-copy_ptr_org[i].angle<<std::endl;
			//std::cout<<Degree2Radians(float((copy_ptr_org[i+1].angle-copy_ptr_org[i].angle)/100))<<std::endl;
			//std::cout<<angle<<std::endl;
			if (angle < min_angle_ || angle > max_angle_)
			{
#if 0
				for (int index = std::max<int>(i - neighbors_, 0); index <= std::min<int>(i + neighbors_, count - 1); index++)
				{
					if (copy_ptr_org[i].distance < copy_ptr_org[index].distance)
						indices_to_delete.insert(index);
				}
#else
				for (int index = 0 - neighbors_; index <= neighbors_; index++)
				{
					int k;
					if (index == 0)  continue;
					k = i + index;
					while (1)
					{
						if ((k < 0)) break;
						if ((k > (count - 1))) break;
						if (copy_ptr_org[k].distance)
						{
							if ((copy_ptr_org[i].distance + distance_std) < copy_ptr_org[k].distance) // delete neighbor if they are farther away (note not self)
							{
								indices_to_delete.insert(k);
								//break;
							}
							break;
						}
						if (index > 0) k++;
						if (index < 0) k--;
					}//end while
				}
#endif
			}
		}
		if ((dandiao1*dandiao2) < 0)
		{
			for (int index = std::max<int>(i - 1, 0); index <= std::min<int>(i + 1, count - 1); index++)
			{

				indices_to_delete.erase(index);
			}
		}

	}
	for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
	{
		copy_ptr[*it].distance = distance_max;  // Failed test to set the ranges to invalid value
	}
	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;
}

pavo_response_scan_t*  MADataFilter::HandlTialFilterWithDistance_2(pavo_response_scan_t* data_buffer, int count)
{
	double dis_diff = 50;
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];

	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	for (int i = 0; i < count; i++)
	{
		if (i == 0 || i == 1)
		{
			continue;
		}

		double dis1 = copy_ptr_org[i].distance;
		double dis2 = copy_ptr_org[i - 2].distance;
		double k = (dis2 - dis1);
		if (k > dis_diff || k < -dis_diff)
		{
			copy_ptr[i].distance = 0;
		}
	}

	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;

}

pavo_response_scan_t*  MADataFilter::HandlTialFilterWithDistance_1(pavo_response_scan_t* data_buffer, int count)
{
	double dis_diff = 5;
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];

	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	for (int i = 0; i < count; i++)
	{
		if (i == 0)
		{
			continue;
		}

		double dis1 = copy_ptr_org[i].distance;
		double dis2 = copy_ptr_org[i - 1].distance;
		double k = (dis2 - dis1);
		if (k > dis_diff || k < -dis_diff)
		{
			copy_ptr[i].distance = 0;
		}
	}

	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;

}

pavo_response_scan_t*  MADataFilter::HandlTialFilterWithRatio(pavo_response_scan_t* data_buffer, int count)
{
	unsigned short angle1, angle2;
	double k_diff = std::tan(80 * M_PI / 180);
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];

	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	for (int i = 0; i < count; i++)
	{
		if (i == 0)
		{
			continue;
		}
		angle1 = copy_ptr_org[i].angle;
		double dis1 = copy_ptr_org[i].distance;
		angle2 = copy_ptr_org[i - 1].angle;
		double dis2 = copy_ptr_org[i - 1].distance;
		double k = (dis2 - dis1) / (angle2 - angle1);
		if (k > k_diff || k < -k_diff)
		{
			copy_ptr[i].distance = 0;
		}
	}

	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;

}

pavo_response_scan_t*  MADataFilter::HandlTialFilter(pavo_response_scan_t* data_buffer, int count)
{
	int start, end, ret;
	pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
	pavo_response_scan_t* copy_ptr_org = new pavo_response_scan_t[count];
	memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
	memcpy(copy_ptr_org, data_buffer, sizeof(pavo_response_scan_t)*count);

	SetWindow(5);
	if (count > 3500)
	{
		SetDistance(50.0);
	}
	if (count <= 3500 && count > 2500)
	{
		SetDistance(100.0);
	}

	for (int i = 0; i < count;)
	{
		start = SearchSkipPoint(copy_ptr_org, i, count);
		if (start >= count) break;
		end = SearchContinuityPoint(copy_ptr_org, start, count);
		ret = HandlePoint(copy_ptr, start, end, count);
		if (ret >= count) break;
		i = ret;
	}
	memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
	delete[] copy_ptr;
	delete[] copy_ptr_org;
	return data_buffer;
}

void MADataFilter::HandlFrontFilter(pavo_response_scan_t* data_buffer,int count)
{
	if(front_filter_level <= 0 || front_filter_level > 3)
		return;

	for(int a = 0; a < count; a++)
	{
		//distanceµ¥Î»Îª0.002m
		if((data_buffer + a)->distance < 500 && (data_buffer + a)->intensity < g_iIntensity_table[front_filter_level - 1])
		{
			(data_buffer + a)->distance = 0;
			(data_buffer + a)->intensity = 0;
		}
	}
}

uint16_t MADataFilter::SearchSkipPoint(pavo_response_scan_t* data_buffer, int idx, int capacity)
{
	int start, count;
	int window_count;
	start = idx;
	count = capacity;
	int i, ret;
	double distance1;
	bool is_skip = true;
	double distance_diff = 0;
	double sum, avg;

	//_window_vector.clear();
	//init the window data
	window_count = 0;
	sum = avg = 0;
	if (start < _window_size)
	{
		for (int j = 0; window_count < _window_size &&j < count; j++)
		{
			ret = j;
			_window_vector[window_count%_window_size] = j;
			if (j >= start && j != 0)
			{
				distance1 = data_buffer[j].distance;
				distance_diff = std::abs(distance1 - avg);
				if (distance_diff > _distance_diff)
				{
					return ret;
				}
			}
			sum += data_buffer[j].distance;
			avg = sum / (window_count + 1);
			window_count++;
		}
	}
	else
	{
		for (int j = start - _window_size; window_count < _window_size &&j < count; j++)
		{
			ret = j;
			_window_vector[window_count%_window_size] = j;
			sum += data_buffer[j].distance;
			avg = sum / (window_count + 1);
			window_count++;
		}
	}
	if (window_count < _window_size) return ret;
	window_count = 0;
	for (i = ret + 1; i < count; i++)
	{
		ret = i;
		distance1 = data_buffer[i].distance;
		distance_diff = std::abs(distance1 - avg);
		if (distance_diff > _distance_diff)
		{
			return ret;
		}
		sum = sum - data_buffer[_window_vector[window_count]].distance;
		sum = sum + data_buffer[i].distance;
		avg = sum / (window_count + 1);
		_window_vector[window_count] = i;
		window_count++;
		window_count %= _window_size;
	}
	return ret;

}

uint16_t MADataFilter::SearchContinuityPoint(pavo_response_scan_t* data_buffer, int idx, int capacity)
{
	int start, count;
	int window_count;
	int i, ret;
	double distance1;
	double distance_diff = 0;
	double sum, avg;
	bool is_continuity = true;

	start = idx;
	count = capacity;
	//_window_vector.clear();
	//init the window data
	window_count = 0;
	sum = avg = 0;
	if (start < _window_size)
	{
		for (int j = 0; window_count < _window_size &&j < count; j++)
		{
			ret = j;
			_window_vector[window_count%_window_size] = j;
			if (j >= start && j != 0)
			{
				distance1 = data_buffer[j].distance;
				distance_diff = std::abs(distance1 - avg);
				if (distance_diff < _distance_diff)
				{
					return ret;
				}
			}
			sum += data_buffer[j].distance;
			avg = sum / (window_count + 1);
			window_count++;
		}
	}
	else
	{
		for (int j = start - _window_size; window_count < _window_size &&j < count; j++)
		{
			ret = j;
			_window_vector[window_count%_window_size] = j;
			sum += data_buffer[j].distance;
			avg = sum / (window_count + 1);
			window_count++;
		}
	}
	if (window_count < _window_size) return ret;
	window_count = 0;
	for (i = ret + 1; i < count; i++)
	{
		ret = i;
		distance1 = data_buffer[i].distance;
		distance_diff = std::abs(distance1 - avg);
		if (distance_diff < _distance_diff)
		{
			return ret;
		}
		sum = sum - data_buffer[_window_vector[window_count]].distance;
		sum = sum + data_buffer[i].distance;
		avg = sum / (window_count + 1);
		_window_vector[window_count] = i;
		window_count++;
		window_count %= _window_size;
	}
	return ret;
}

uint16_t MADataFilter::HandlePoint(pavo_response_scan_t* data_buffer, int start, int end, int capacity)
{
	int i;
	int count = capacity;
	int half = (start + end) / 2;
	for (i = start; i < end && i < count; i++)
	{
		if (data_buffer[i].distance >= 1500)
		{
			continue;
		}
		if (i < half)
		{
			if (start >= 1)
			{
				data_buffer[i].distance = data_buffer[start - 1].distance;
			}
			else
			{
				data_buffer[i].distance = 0;
			}
		}
		else
		{
			if (end >= 1)
			{
				data_buffer[i].distance = data_buffer[end].distance;
			}
			else
			{
				data_buffer[i].distance = 0;
			}
		}
	}
	return end + 1;

}

void MADataFilter::SetWindow(int size)
{
	if (size)
	{
		_window_size = size;
		_window_vector.resize(size);
	}
	else
	{
		_window_size = 5;
		_window_vector.resize(5);
	}

}

void MADataFilter::SetDistance(double size)
{
	if (size)
	{
		_distance_diff = size;
	}
	else
	{
		_distance_diff = 5.0;
	}

}

void MADataFilter::SetIntensity(double size)
{
	_intensity_diff = size;
}

void MADataFilter::EnableTailFilter(int method)
{
	tailfilter_method_angle = method;
}

void MADataFilter::EnableFrontFilter(int level)
{
	front_filter_level = level;
}
