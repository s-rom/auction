#ifndef POINT_2D
#define POINT_2D

#include <string>
#include <cmath>

namespace Auction
{
	using std::string;
	struct Point2D;
}

struct Auction::Point2D
{
	float x, y;

	/**
	 * Constructor 
	 * Requires initialization for x and y values
	 */
	Point2D(float x, float y)
	: 
		x(x),
		y(y)
	{}

	/**
	 * Default constructor, sets x and y to (0,0) 
	 */
	Point2D()
	:
		x(0),
		y(0)
	{}


	/**
	 * Returns a string representing the point
	 * 	Format: (x,y)
	 */
	string to_string()
	{
		return "(" + std::to_string(this->x) + "," + std::to_string(this->y) + ")";
	}

	/**
	 * Serializes the point, used by the MessageSystem 
	 * 	Format (being delim #): x#y
	 */
	string serialize(char delim)
	{
		std::string s;
		s = std::to_string(x) + delim + std::to_string(y);
		return s;
	}

	/**
	 * Returns the euclidean distance between two points.
	 */ 
	static float euclidean_distance(Point2D p1, Point2D p2)
	{
		using std::pow;
		using std::sqrt;
		float in_term = pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2);
		if (in_term >= 0) return sqrt(in_term);
		else return 0;
	}

	Point2D& operator=(const Point2D& other)
	{
		this->x = other.x;
		this->y = other.y;
		return *this;
	}

};

#endif