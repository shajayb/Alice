#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

class circle
{
public:
	vector<int> ids;
	circle() {}; // empty constructor;


	circle(newPhysics &PP , float r , vec cen )
	{
		vec pt;
		ids.clear();

		for (int i = 0; i < 64; i++)
		{
			pt.y = r * sin(2 * PI / 64 * i);
			pt.x = r * cos(2 * PI / 64 * i);
			pt += cen;
			int n = PP.makeParticle(pt, 1.0, false);
			ids.push_back(n);
			PP.p[n].v = PP.p[n].p - cen;
			PP.p[n].v.normalise();
		}
	}

	void checkCollisions( circle &other , newPhysics &phy )
	{
		for (int i = 0; i < ids.size(); i++)
		{
			int idOfbox = i;
			int idOfparticle = ids[idOfbox];
			vec a = phy.p[idOfparticle].p;
				
				for (int j = 0; j < other.ids.size(); j++)
				{
					int idOfbox_o = j;
					int idOfparticle_o = other.ids[idOfbox_o];
					vec b = phy.p[idOfparticle_o].p;

					float distance = (b - a).mag();

					if (distance < 1.0)
					{
						phy.p[idOfparticle].fixed = true;
						phy.p[idOfparticle_o].fixed = true;
					}
				}
		}
	}

	void draw( newPhysics &phy)
	{
		for (int i = 1; i < ids.size(); i++)
		{
			int idOfCurrentPoint = ids[i];
			int idOfPreviousPoint = ids[i-1];
			drawLine(phy.p[idOfCurrentPoint].p, phy.p[idOfPreviousPoint].p);
		}
	}
};