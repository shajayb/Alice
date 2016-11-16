#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "graph.h"


class newPhysics : public physics
{

public:

	newPhysics()
	{
		calcCharge = false;
		calcCustom = true;
		down = vec(0, 0, 0.0);
		gravity = down.mag();
	}

	virtual void calcCustomForces_post(PARTICLE *p)
	{
		for (int i = 0; i < np; i++)
			for (int j = 0; j < np; j++)
			{
				if (i == j)continue;

				vec f = p[i].p - p[j].p;
				double d = f*f;

				f.normalise();
				if (d >= 1e-8 && d < 18)
				{

					if (!p[i].fixed)p[i].f += (f * 1) / d;
					if (!p[j].fixed)p[j].f -= (f * 1) / d;
				}
			
			}
	}


};
#pragma once
