// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <ScalarField.h>

//System
#include <cassert>
#include <cstring>
#include <algorithm>

using namespace CCCoreLib;

ScalarField::ScalarField(const char* name/*=nullptr*/)
{
	setName(name);
}

ScalarField::ScalarField(const ScalarField& sf)
	: std::vector<ScalarType>(sf)
{
	setName(sf.m_name);
}

void ScalarField::setName(const char* name)
{
	if (name)
		strncpy(m_name, name, 255);
	else
		strcpy(m_name, "Undefined");
}

ScalarType ScalarField::getValueByPercent(double percent)
{
    std::vector<ScalarType> copydata(begin(),end());
    int index = copydata.size() * percent;
    nth_element(copydata.begin(), copydata.begin() + index, copydata.end());
    if (index < copydata.size())
    {
        return copydata[index];
    }
    else
    {
        return 0;
    }
}

void ScalarField::getValueByPercentList(const std::vector<double> & percentlist, std::vector<ScalarType> & resultlist)
{
    std::vector<ScalarType> copydata(begin(), end());
    //std::sort(copydata.begin(), copydata.end());
    resultlist.clear();
    for (auto cutpercent : percentlist)
    {
        int index = copydata.size() * cutpercent;
        if (index < copydata.size())
        {
            nth_element(copydata.begin(), copydata.begin() + index, copydata.end());
            resultlist.push_back(copydata[index]);
        }
        else
        {
            resultlist.push_back(0);
        }
    }
}

void ScalarField::computeMeanAndVariance(ScalarType &mean, ScalarType* variance) const
{
	double _mean = 0.0;
	double _std2 = 0.0;
	std::size_t count = 0;

	for (std::size_t i = 0; i < size(); ++i)
	{
		const ScalarType& val = at(i);
		if (ValidValue(val))
		{
			_mean += val;
			_std2 += static_cast<double>(val) * val;
			++count;
		}
	}

	if (count)
	{
		_mean /= count;
		mean = static_cast<ScalarType>(_mean);

		if (variance)
		{
			_std2 = std::abs(_std2 / count - _mean*_mean);
			*variance = static_cast<ScalarType>(_std2);
		}
	}
	else
	{
		mean = 0;
		if (variance)
		{
			*variance = 0;
		}
	}
}

bool ScalarField::reserveSafe(std::size_t count)
{
	try
	{
		reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

bool ScalarField::resizeSafe(std::size_t count, bool initNewElements/*=false*/, ScalarType valueForNewElements/*=0*/)
{
	try
	{
		if (initNewElements)
			resize(count, valueForNewElements);
		else
			resize(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}
