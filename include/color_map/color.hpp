#ifndef COLOR_HPP
#define COLOR_HPP

#include <tuple>
#include <memory>

namespace color_map
{

static float clamp(float val, float min, float max)
{
	if (val <= min) return min;
	if (val >= max) return max;

	return val;
}

const constexpr float est_temp_min = 50.0f;
const constexpr float est_temp_max = 100.0f;
const constexpr float est_temp_range = est_temp_max - est_temp_min;

class ColorHandler
{

	float raw_temp;
	
	struct Colors
	{
		float r, g, b;
	};

	class ColorHandlerImpl;
	std::unique_ptr<ColorHandlerImpl> pImpl;


public:
	ColorHandler(float temp);

	std::tuple<float, float, float> get_rgb();

};


};


#endif